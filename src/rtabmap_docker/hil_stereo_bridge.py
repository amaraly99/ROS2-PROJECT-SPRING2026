#!/usr/bin/env python3
"""HIL stereo bridge for the RTAB-Map sidecar (created 2026-09-08).

One rclpy node that closes the three gaps between rtabmap_odom/stereo_odometry and
this project's backend-agnostic SLAM contract (docs/fixlog/README.md, run_matrix.py):

1. CameraInfo. The HIL camera bridges (ovcam_bridge, ovcam_bridge_right) publish only
   images. RTAB-Map's stereo pipeline needs left/right CameraInfo whose header stamp
   EXACTLY matches the image (exact sync, approx_sync: false). This node subscribes
   both /ovcam/* images and republishes a CameraInfo per image with the image's own
   stamp and frame_id. Intrinsics are the verified simulator model (fx=fy=554, cx=320,
   cy=240, zero distortion -- docs/fixlog/027-*), baseline 0.11 m encoded the way
   rtabmap reads it: right P[3] = -fx * baseline (Tx). Modeled on euroc_caminfo_pub.py.

2. /slam/pose (geometry_msgs/PoseStamped). stereo_odometry publishes nav_msgs/Odometry
   on `odom` (odom frame). When the rtabmap SLAM node has published the map->odom
   correction on TF, the pose is expressed in `map` so it is the loop-corrected live
   pose, comparable to what OV2SLAM / ORB-SLAM publish; before that (or if TF is
   unavailable) the raw odometry pose is published.

3. /slam/tracking_state (std_msgs/Int32, 2 = tracking) and the per-frame timing CSV.
   run_matrix.py polls tracking_state to decide whether SLAM initialised, and its
   generic readiness gate waits for timing_events.csv to GROW; aggregate_matrix.py
   reads the same file (category frontend/full_tracking) for front-end latency. Rows
   follow the ORB-SLAM2 BenchmarkUtils schema
   (wall_ts,thread_role,category,duration_ms,frame_id,keyframe_id,tracking_state)
   with duration = OdomInfo.time_estimation (s) * 1000. Path from ORB_BENCH_TIMING_CSV
   (exported by run_stack_hil.sh in benchmark mode); disabled if unset.

State mapping (rtabmap has no ORB-style state enum): 1 until the first odometry
message, then 2 while OdomInfo.lost is false and the pose is non-null, 3 when lost.
"""
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32

import tf2_ros

FX = 554.0
FY = 554.0
CX = 320.0
CY = 240.0
WIDTH = 640
HEIGHT = 480
BASELINE_M = 0.11

STATE_NOT_INIT = 1
STATE_OK = 2
STATE_LOST = 3


def make_camera_info(header, tx):
    msg = CameraInfo()
    msg.header.stamp = header.stamp
    msg.header.frame_id = header.frame_id
    msg.width = WIDTH
    msg.height = HEIGHT
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [FX, 0.0, CX, tx, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def quat_to_mat(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)


class HilStereoBridge(Node):
    def __init__(self):
        super().__init__("hil_stereo_bridge")
        self.declare_parameter("left_image_topic", "/ovcam/image_raw")
        self.declare_parameter("right_image_topic", "/ovcam/right/image_raw")
        self.declare_parameter("left_info_topic", "/ovcam/camera_info")
        self.declare_parameter("right_info_topic", "/ovcam/right/camera_info")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("odom_info_topic", "odom_info")
        self.declare_parameter("pose_topic", "/slam/pose")
        self.declare_parameter("state_topic", "/slam/tracking_state")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("use_map_correction", True)
        self.declare_parameter("timing_csv", os.environ.get("ORB_BENCH_TIMING_CSV", ""))

        p = lambda k: self.get_parameter(k).value
        self.map_frame = str(p("map_frame"))
        self.odom_frame = str(p("odom_frame"))
        self.use_map_correction = bool(p("use_map_correction"))

        self.left_info_pub = self.create_publisher(CameraInfo, str(p("left_info_topic")), qos_profile_sensor_data)
        self.right_info_pub = self.create_publisher(CameraInfo, str(p("right_info_topic")), qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, str(p("pose_topic")), 10)
        self.state_pub = self.create_publisher(Int32, str(p("state_topic")), 10)

        # ovcam_bridge publishes RELIABLE; a best-effort (sensor-data) subscriber is
        # compatible with that and never applies back-pressure to the camera path.
        self.create_subscription(Image, str(p("left_image_topic")), self._left_cb, qos_profile_sensor_data)
        self.create_subscription(Image, str(p("right_image_topic")), self._right_cb, qos_profile_sensor_data)
        # stereo_odometry publishes odom / odom_info with the launch's qos=2 (BEST_EFFORT).
        # A RELIABLE subscriber does not match a best-effort publisher (DDS QoS rule), so
        # these MUST be best-effort too -- found live on the wiring test (2026-09-08).
        # A best-effort subscriber also matches a reliable publisher, so this is safe
        # whichever way the launch is configured.
        self.create_subscription(Odometry, str(p("odom_topic")), self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(OdomInfo, str(p("odom_info_topic")), self._odom_info_cb, qos_profile_sensor_data)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state = STATE_NOT_INIT
        self.lost = False
        self.frame_id = 0
        self.keyframe_id = 0
        self.n_left = 0
        self.n_right = 0
        self.n_odom = 0
        self.n_corrected = 0

        self.timing_path = str(p("timing_csv"))
        self.timing_fh = None
        if self.timing_path:
            new_file = not os.path.exists(self.timing_path) or os.path.getsize(self.timing_path) == 0
            self.timing_fh = open(self.timing_path, "a", buffering=1)
            if new_file:
                self.timing_fh.write("wall_ts,thread_role,category,duration_ms,frame_id,keyframe_id,tracking_state\n")
                self.timing_fh.flush()
            self.get_logger().info(f"timing CSV -> {self.timing_path}")
        else:
            self.get_logger().info("ORB_BENCH_TIMING_CSV unset -- timing CSV disabled (scout mode)")

        self.create_timer(5.0, self._report)
        # 1 Hz heartbeat of the current state: run_matrix.py reads the topic with
        # `ros2 topic echo --once` and treats an empty read as "retry", so the state must
        # be observable even before odometry has produced anything (state 1).
        self.create_timer(1.0, self._publish_state)
        self.get_logger().info(
            "hil_stereo_bridge up: images %s + %s -> camera_info; odom -> %s, state -> %s"
            % (p("left_image_topic"), p("right_image_topic"), p("pose_topic"), p("state_topic")))

    # ── CameraInfo per image, same stamp/frame ─────────────────────────────
    def _left_cb(self, img):
        self.n_left += 1
        self.left_info_pub.publish(make_camera_info(img.header, 0.0))

    def _right_cb(self, img):
        self.n_right += 1
        self.right_info_pub.publish(make_camera_info(img.header, -FX * BASELINE_M))

    # ── odometry -> /slam/pose (+ map correction) ───────────────────────────
    def _odom_cb(self, odom):
        self.n_odom += 1
        q = odom.pose.pose.orientation
        t = odom.pose.pose.position
        null_pose = (t.x == 0.0 and t.y == 0.0 and t.z == 0.0 and q.w == 0.0) or any(
            math.isnan(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w))
        if null_pose:
            self.state = STATE_LOST
            self._publish_state()
            return

        self.state = STATE_LOST if self.lost else STATE_OK
        R = quat_to_mat(q.x, q.y, q.z, q.w)
        tvec = np.array([t.x, t.y, t.z])
        frame = odom.header.frame_id or self.odom_frame

        if self.use_map_correction:
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, frame, rclpy.time.Time())
                tq, tt = tf.transform.rotation, tf.transform.translation
                Rm = quat_to_mat(tq.x, tq.y, tq.z, tq.w)
                tvec = Rm @ tvec + np.array([tt.x, tt.y, tt.z])
                R = Rm @ R
                frame = self.map_frame
                self.n_corrected += 1
            except Exception:
                pass  # rtabmap has not published map->odom yet: raw odometry pose

        qx, qy, qz, qw = mat_to_quat(R)
        ps = PoseStamped()
        ps.header.stamp = odom.header.stamp
        ps.header.frame_id = frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in tvec)
        ps.pose.orientation.x, ps.pose.orientation.y = float(qx), float(qy)
        ps.pose.orientation.z, ps.pose.orientation.w = float(qz), float(qw)
        self.pose_pub.publish(ps)
        self._publish_state()

    # ── OdomInfo -> lost flag + timing row ──────────────────────────────────
    def _odom_info_cb(self, info):
        self.lost = bool(info.lost)
        if self.state != STATE_NOT_INIT:
            self.state = STATE_LOST if self.lost else STATE_OK
        self.frame_id += 1
        if info.key_frame_added:
            self.keyframe_id += 1
        if self.timing_fh is not None:
            self.timing_fh.write("%.6f,RtabmapOdom,frontend/full_tracking,%.3f,%d,%d,%d\n" % (
                time.time(), float(info.time_estimation) * 1000.0,
                self.frame_id, self.keyframe_id, self.state))
            self.timing_fh.flush()
        self._publish_state()

    def _publish_state(self):
        m = Int32()
        m.data = int(self.state)
        self.state_pub.publish(m)

    def _report(self):
        self.get_logger().info(
            "left=%d right=%d odom=%d map_corrected=%d frames=%d kf=%d state=%d lost=%s"
            % (self.n_left, self.n_right, self.n_odom, self.n_corrected,
               self.frame_id, self.keyframe_id, self.state, self.lost))


def main():
    rclpy.init()
    node = HilStereoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.timing_fh is not None:
            node.timing_fh.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
