#!/usr/bin/env python3
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolo_msgs.msg import Detection
from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.get_logger().info('Loading YOLO Model...')
        self.model = YOLO('yolo26n.pt')

        # ---- knobs ----
        self.imgsz = 320
        self.min_conf = 0.25          # 0.10 is noisy + more boxes = more work
        self.target_class = "person"  # now actually used
        self.yolo_hz = 12.0           # realistic. set near what you want, not 50.
        self.annotated_hz = 5.0       # do NOT stream 1:1 with inference
        self.publish_annotated = True # flip off for max FPS

        # ---- ROS I/O ----
        self.subscription = self.create_subscription(
            Image, 'image_raw', self.image_callback, qos_profile_sensor_data
        )
        self.publisher_ = self.create_publisher(
            Detection, '/yolo/detection', qos_profile_sensor_data
        )
        self.pub_annotated = self.create_publisher(
            Image, '/yolo/annotated', qos_profile_sensor_data
        )
        self.bridge = CvBridge()

        # ---- shared latest frame (drop old frames) ----
        self._lock = threading.Lock()
        self._latest_msg = None
        self._latest_frame = None
        self._latest_stamp = 0.0

        # ---- timers ----
        self.last_annotated_pub_t = 0.0

        # FPS tracking of inference loop
        self._inf_last_t = time.time()
        self._inf_frames = 0

        # run YOLO in timer, not callback
        self.create_timer(1.0 / self.yolo_hz, self.process_latest)

        self.get_logger().info('Detector Node Online!')

    def image_callback(self, msg: Image):
        # Convert in callback OR store msg and convert later.
        # Converting here is fine; we drop frames anyway.
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self._lock:
            self._latest_msg = msg
            self._latest_frame = frame
            self._latest_stamp = time.time()

    def process_latest(self):
        with self._lock:
            if self._latest_frame is None:
                return
            frame = self._latest_frame
            msg = self._latest_msg

        # ---- inference ----
        # classes filtering reduces postprocess work when you only care about one class
        # (works if class name exists in model.names)
        class_id = None
        for k, v in self.model.names.items():
            if v == self.target_class:
                class_id = k
                break

        results = self.model.predict(
            source=frame,
            imgsz=self.imgsz,
            conf=self.min_conf,
            classes=[class_id] if class_id is not None else None,
            verbose=False
        )
        r = results[0]

        # FPS tracking
        now = time.time()
        self._inf_frames += 1
        if now - self._inf_last_t >= 1.0:
            fps = self._inf_frames / (now - self._inf_last_t)
            self.get_logger().info(f"YOLO Inference FPS: {fps:.1f}")
            self._inf_frames = 0
            self._inf_last_t = now

        best_box = None
        best_conf = 0.0
        best_name = None

        boxes = r.boxes
        if boxes is not None and len(boxes) > 0:
            # fastest: just take max confidence directly
            # but ultralytics keeps boxes as a list-like; loop is ok for few boxes
            for b in boxes:
                conf = float(b.conf[0])
                if conf > best_conf:
                    best_conf = conf
                    best_box = b
                    cls_id = int(b.cls[0])
                    best_name = r.names[cls_id]

        # publish detection
        if best_box is not None:
            det_msg = Detection()
            det_msg.class_name = best_name
            det_msg.confidence = float(best_conf)

            cx, cy, w, h = best_box.xywhn[0].tolist()
            det_msg.center_x = float(cx)
            det_msg.center_y = float(cy)
            self.publisher_.publish(det_msg)

        # publish annotated at a lower rate
        if self.publish_annotated:
            if now - self.last_annotated_pub_t >= 1.0 / self.annotated_hz:
                self.last_annotated_pub_t = now
                annotated = frame.copy()
                if best_box is not None:
                    x1, y1, x2, y2 = best_box.xyxy[0].tolist()
                    x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                    label = f"{best_name} {best_conf:.2f}"
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated, label, (x1, max(0, y1 - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    cv2.putText(annotated, "NO TARGET DETECTED", (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                annotated_msg.header = msg.header
                self.pub_annotated.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
