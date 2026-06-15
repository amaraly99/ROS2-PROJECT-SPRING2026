#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
# oracle_detector_node.py
#
# YOLO-AGNOSTIC detector. Instead of looking at the image and guessing,
# it COMPUTES the perfect bounding box by projecting the known target
# world point through the sim camera using ground-truth drone pose.
# Publishes on /yolo/detections (same topic + type as the real YOLO
# bridge) so the controller cannot tell the difference — but the
# detection is now perfect and identical every run.
#
# This removes the detector as a confounding variable: any TS1 (IBVS) vs
# TS2 (proportional) difference is the servoing law, not detector noise.
#
# Inputs  : /sim/drone_pose [x,y,z,pitch,yaw]  (Float64MultiArray)
#           /sim/target_pose [x,y,z,yaw]        (Float64MultiArray, latched)
# Output  : /yolo/detections                    (yolo_msgs/DetectionArray)
#
# The box SHRINKS when far and GROWS when near (size_height = fy*H/Zc),
# exactly as a real detector would see.
#
# ⚠ AXIS CONVENTION (must be validated against the .slx with live view):
#   body frame   x=forward, y=LEFT, z=UP  (Simulink confirmed in node header)
#   camera frame Zc=forward, Xc=right, Yc=down  (standard CV pinhole)
#   pitch_sign flips nose-down direction if the box lands inverted.
# ─────────────────────────────────────────────────────────────────
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64, Float64MultiArray
from yolo_msgs.msg import Detection, DetectionArray


class OracleDetector(Node):
    def __init__(self):
        super().__init__('oracle_detector')

        # ── Parameters ──
        self.declare_parameter('cam_fx', 554.0)
        self.declare_parameter('cam_fy', 554.0)
        self.declare_parameter('cam_cx', 320.0)
        self.declare_parameter('cam_cy', 240.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('known_target_height', 1.5)   # metres
        self.declare_parameter('target_aspect', 1.0)         # box w/h (MUST match IBVS)
        self.declare_parameter('target_class', 'stop sign')
        self.declare_parameter('confidence', 0.99)
        self.declare_parameter('publish_rate', 20.0)         # Hz
        self.declare_parameter('pitch_sign', 1.0)            # flip if box inverts
        self.declare_parameter('edge_margin_px', 0.0)        # off-screen guard
        # Phase-2 detector-stress knobs (default 0 = perfect oracle):
        self.declare_parameter('pixel_noise_std', 0.0)       # px gaussian on centre
        self.declare_parameter('dropout_prob', 0.0)          # per-frame miss prob

        self.fx = self.get_parameter('cam_fx').value
        self.fy = self.get_parameter('cam_fy').value
        self.cx0 = self.get_parameter('cam_cx').value
        self.cy0 = self.get_parameter('cam_cy').value
        self.W = int(self.get_parameter('image_width').value)
        self.H = int(self.get_parameter('image_height').value)
        self.known_h = self.get_parameter('known_target_height').value
        self.aspect = self.get_parameter('target_aspect').value
        self.cls = self.get_parameter('target_class').value
        self.conf = self.get_parameter('confidence').value
        rate = self.get_parameter('publish_rate').value
        self.pitch_sign = self.get_parameter('pitch_sign').value
        self.edge_margin = self.get_parameter('edge_margin_px').value
        self.noise_std = self.get_parameter('pixel_noise_std').value
        self.dropout = self.get_parameter('dropout_prob').value

        # ── State ──
        self.drone = None     # (x,y,z,pitch,yaw)
        self.target = None    # (x,y,z,yaw)

        # ── QoS (match the rest of the graph) ──
        pose_qos = QoSProfile(depth=10,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST)
        tgt_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        det_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST)

        self.sub_drone = self.create_subscription(
            Float64MultiArray, '/sim/drone_pose', self.on_drone, pose_qos)
        self.sub_target = self.create_subscription(
            Float64MultiArray, '/sim/target_pose', self.on_target, tgt_qos)
        self.pub = self.create_publisher(DetectionArray, '/yolo/detections', det_qos)

        self.timer = self.create_timer(1.0 / rate, self.tick)
        self._last_log = self.get_clock().now()

        self.get_logger().info(
            f"oracle_detector up — class='{self.cls}' fx={self.fx} "
            f"H_known={self.known_h}m aspect={self.aspect} rate={rate}Hz "
            f"(noise={self.noise_std}px dropout={self.dropout})")

    def on_drone(self, msg):
        if len(msg.data) >= 5:
            self.drone = tuple(msg.data[:5])   # x,y,z,pitch,yaw

    def on_target(self, msg):
        if len(msg.data) >= 4:
            self.target = tuple(msg.data[:4])  # x,y,z,yaw

    def project(self):
        """Return (u, v, bw, bh) in pixels, or None if target not visible."""
        if self.drone is None or self.target is None:
            return None
        x, y, z, pitch, yaw = self.drone
        tx, ty, tz, _ = self.target

        # Vector drone→target in world.
        dx, dy, dz = tx - x, ty - y, tz - z

        # World → body (x=forward, y=left, z=up) by yaw about world Z.
        cyaw, syaw = math.cos(yaw), math.sin(yaw)
        fwd = dx * cyaw + dy * syaw       # along heading
        left = -dx * syaw + dy * cyaw     # +y left
        up = dz                           # +z up

        # Apply pitch about the body lateral axis (nose-down positive).
        th = self.pitch_sign * pitch
        cth, sth = math.cos(th), math.sin(th)
        fwd_p = fwd * cth + up * sth
        up_p = -fwd * sth + up * cth

        # Body → camera optical (Zc forward, Xc right, Yc down).
        Zc = fwd_p
        Xc = -left      # right is -left
        Yc = -up_p      # down is -up
        if Zc <= 0.1:
            return None                   # behind the camera

        u = self.fx * (Xc / Zc) + self.cx0
        v = self.fy * (Yc / Zc) + self.cy0

        m = self.edge_margin
        if u < -m or u > self.W + m or v < -m or v > self.H + m:
            return None                   # off-screen

        bh = self.fy * self.known_h / Zc  # grows as Zc shrinks
        bw = bh * self.aspect
        return (u, v, bw, bh)

    def tick(self):
        import random
        proj = self.project()
        out = DetectionArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'camera'

        if proj is not None and random.random() >= self.dropout:
            u, v, bw, bh = proj
            if self.noise_std > 0.0:
                u += random.gauss(0.0, self.noise_std)
                v += random.gauss(0.0, self.noise_std)
            d = Detection()
            d.class_name = self.cls
            d.confidence = float(self.conf)
            d.center_x = float(u)
            d.center_y = float(v)
            d.size_width = float(bw)
            d.size_height = float(bh)
            out.detections.append(d)

        self.pub.publish(out)

        # 1 Hz heartbeat log so you can see the oracle is alive + box size.
        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds > 1e9:
            self._last_log = now
            if proj is not None:
                u, v, bw, bh = proj
                self.get_logger().info(
                    f"[oracle] box centre=({u:.0f},{v:.0f}) size={bw:.0f}x{bh:.0f}px "
                    f"(grows as it nears)")
            else:
                self.get_logger().info("[oracle] target NOT visible → empty (drives SEARCH)")


def main():
    rclpy.init()
    node = OracleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()