#!/usr/bin/env python3
"""HIL RGB-D bridge for the RTAB-Map sidecar (created 2026-09-27).

Copy of hil_stereo_bridge.py for RGB-D (src/rtabmap_docker/hil_rgbd.launch.py), with
the right eye removed. It still closes the same gaps between rtabmap_odom/rgbd_odometry
and this project's backend-agnostic SLAM contract:

1. CameraInfo for the LEFT image only (/ovcam/camera_info), same stamp and frame_id as
   each image. Depth (/sim/camera/depth/image_raw) comes straight from the RGB-D
   Simulink model, registered to the left camera, on the same MATLAB stamp.
   Intrinsics are parameters (fx, fy, cx, cy) defaulting to the verified stereo-block
   model (fx=fy=554, cx=320, cy=240 -- FIX-027); the RGB-D model's left camera must
   match them or the real values must be passed here.
2. /slam/pose (PoseStamped): odometry pose, expressed in `map` once rtabmap publishes
   map->odom (loop-corrected live pose), exactly as the stereo bridge.
3. /slam/tracking_state (Int32, 2 = tracking) and the per-frame timing CSV
   (frontend/full_tracking = OdomInfo.time_estimation), exactly as the stereo bridge.
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

STATE_NOT_INIT = 1
STATE_OK = 2
STATE_LOST = 3


def make_camera_info(header, tx, fx=FX, fy=FY, cx=CX, cy=CY):
    msg = CameraInfo()
    msg.header.stamp = header.stamp
    msg.header.frame_id = header.frame_id
    msg.width = WIDTH
    msg.height = HEIGHT
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, tx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
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


class HilRgbdBridge(Node):
    def __init__(self):
        super().__init__("hil_rgbd_bridge")
        self.declare_parameter("left_image_topic", "/ovcam/image_raw")
        self.declare_parameter("left_info_topic", "/ovcam/camera_info")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("odom_info_topic", "odom_info")
        self.declare_parameter("pose_topic", "/slam/pose")
        self.declare_parameter("state_topic", "/slam/tracking_state")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("use_map_correction", True)
        self.declare_parameter("timing_csv", os.environ.get("ORB_BENCH_TIMING_CSV", ""))
        self.declare_parameter("fx", FX)
        self.declare_parameter("fy", FY)
        self.declare_parameter("cx", CX)
        self.declare_parameter("cy", CY)

        p = lambda k: self.get_parameter(k).value
        self.map_frame = str(p("map_frame"))
        self.odom_frame = str(p("odom_frame"))
        self.use_map_correction = bool(p("use_map_correction"))
        self.K = dict(fx=float(p("fx")), fy=float(p("fy")), cx=float(p("cx")), cy=float(p("cy")))

        self.left_info_pub = self.create_publisher(CameraInfo, str(p("left_info_topic")), qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, str(p("pose_topic")), 10)
        self.state_pub = self.create_publisher(Int32, str(p("state_topic")), 10)

        # ovcam_bridge publishes RELIABLE; a best-effort (sensor-data) subscriber is
        # compatible with that and never applies back-pressure to the camera path.
        self.create_subscription(Image, str(p("left_image_topic")), self._left_cb, qos_profile_sensor_data)
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
            "hil_rgbd_bridge up: image %s -> camera_info (fx=%.1f fy=%.1f cx=%.1f cy=%.1f); odom -> %s, state -> %s"
            % (p("left_image_topic"), self.K["fx"], self.K["fy"], self.K["cx"], self.K["cy"],
               p("pose_topic"), p("state_topic")))

    # ── CameraInfo per image, same stamp/frame ─────────────────────────────
    def _left_cb(self, img):
        self.n_left += 1
        self.left_info_pub.publish(make_camera_info(img.header, 0.0, **self.K))


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
            "left=%d odom=%d map_corrected=%d frames=%d kf=%d state=%d lost=%s"
            % (self.n_left, self.n_odom, self.n_corrected,
               self.frame_id, self.keyframe_id, self.state, self.lost))


def main():
    rclpy.init()
    node = HilRgbdBridge()
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
