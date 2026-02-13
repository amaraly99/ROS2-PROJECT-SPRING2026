#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()

        # Use explicit device + backend (better in VirtualBox)
        self.cap = cv2.VideoCapture('rtsp://host.docker.internal:8554/live', cv2.CAP_FFMPEG)

        # Force MJPG + low resolution + low fps + minimal buffering
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Camera Online: MJPG negotiated @ {w}x{h}, 15fps target')

        # publish at 15Hz
        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)

    def timer_callback(self):
        # Drain stale buffered frames to reduce latency in VM
        for _ in range(3):
            self.cap.grab()

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Frame dropped.")
            return

        frame = cv2.resize(frame, (320, 240))
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
