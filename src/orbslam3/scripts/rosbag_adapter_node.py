#!/usr/bin/env python3

"""
Direct EuRoC rosbag adapter for the ROS2 ORB-SLAM3 wrapper.

This node reads the EuRoC rosbag in-process via rosbag2_py, starts the
configuration handshake only after the first frame or stereo pair is observed,
buffers the latest pending input until the wrapper ACKs initialization, then
streams images directly to the wrapper topics and performs an ACK-based
shutdown once the bag is exhausted.
"""

import threading
import time
from enum import Enum
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

SUPPORTED_BAG_SUFFIXES = (".db3", ".mcap")


class AdapterState(Enum):
    WAIT_FOR_ORBSLAM = "wait_for_orbslam"
    WAIT_FOR_FIRST_FRAME = "wait_for_first_frame"
    HANDSHAKE_ACTIVE = "handshake_active"
    STREAMING = "streaming"
    SHUTDOWN_PENDING = "shutdown_pending"
    DONE = "done"


class RosbagAdapter(Node):
    def __init__(self):
        super().__init__("rosbag_adapter_node")

        self.declare_parameter("mode", "mono")
        self.declare_parameter("settings_name", "EuRoC")
        self.declare_parameter("bag_dir", "datasets/euroc/MH_05_difficult")
        self.declare_parameter("rectify_stereo", False)
        self.declare_parameter("rectify_settings_file", "")
        self.declare_parameter("shutdown_drain_sec", 1.0)
        self.declare_parameter("shutdown_timeout_sec", 10.0)
        self.declare_parameter("playback_rate", 1.0)
        self.declare_parameter("handshake_timeout_sec", 120.0)

        self.mode = str(self.get_parameter("mode").value).lower()
        self.settings_name = str(self.get_parameter("settings_name").value)
        rectify_value = self.get_parameter("rectify_stereo").value
        if isinstance(rectify_value, str):
            self.rectify_stereo = rectify_value.strip().lower() in {"true", "1", "yes", "on"}
        else:
            self.rectify_stereo = bool(rectify_value)
        self.shutdown_drain_sec = float(self.get_parameter("shutdown_drain_sec").value)
        self.shutdown_timeout_sec = float(self.get_parameter("shutdown_timeout_sec").value)
        self.playback_rate = float(self.get_parameter("playback_rate").value)
        self.handshake_timeout_sec = float(self.get_parameter("handshake_timeout_sec").value)

        if self.mode not in {"mono", "stereo"}:
            raise ValueError("mode must be either 'mono' or 'stereo'")
        if self.playback_rate <= 0.0:
            raise ValueError("playback_rate must be > 0")

        self.repo_root = Path(__file__).resolve().parents[2]
        raw_bag_dir = Path(str(self.get_parameter("bag_dir").value)).expanduser()
        raw_rectify_settings = Path(str(self.get_parameter("rectify_settings_file").value)).expanduser()
        self.bag_dir = raw_bag_dir if raw_bag_dir.is_absolute() else (self.repo_root / raw_bag_dir)
        self.bag_dir = self.bag_dir.resolve()
        if not self.bag_dir.exists():
            raise FileNotFoundError(f"Bag directory does not exist: {self.bag_dir}")

        self.rectify_settings_file = None
        if self.mode == "stereo" and self.rectify_stereo:
            candidate = raw_rectify_settings if raw_rectify_settings.is_absolute() else (self.repo_root / raw_rectify_settings)
            self.rectify_settings_file = candidate.resolve()
            if not self.rectify_settings_file.exists():
                raise FileNotFoundError(
                    f"Rectification settings file does not exist: {self.rectify_settings_file}"
                )

        self.output_prefix = "/mono_py_driver" if self.mode == "mono" else "/stereo_py_driver"
        self.input_image_topic = "/cam0/image_raw"
        self.left_image_topic = "/cam0/image_raw"
        self.right_image_topic = "/cam1/image_raw"

        self.experiment_settings_payload = f"{self.settings_name}|{self.bag_dir.name}"
        self.shutdown_service_name = f"{self.output_prefix}/shutdown_request"

        self.state = AdapterState.WAIT_FOR_ORBSLAM
        self.config_acked = False
        self.config_ack_event = threading.Event()
        self.publish_lock = threading.Lock()
        self.last_published_stamp_sec = None
        self._pace_wall_anchor = time.monotonic()
        self.first_frame_seen = False
        self.config_send_count = 0
        self.last_config_log_time = None
        self.orbslam_ready_count = 0      # consecutive ticks where orbslam_available() == True
        self.bag_exit_code = None
        self.bag_exit_monotonic = None
        self.shutdown_request_sent = False
        self.shutdown_request_monotonic = None
        self.shutdown_future = None
        self.reader_thread = None
        self.stop_event = threading.Event()
        self.pending_mono_msg = None
        self.pending_left_msg = None
        self.pending_right_msg = None
        self.bridge = CvBridge()
        self.left_rectify_map_1 = None
        self.left_rectify_map_2 = None
        self.right_rectify_map_1 = None
        self.right_rectify_map_2 = None

        if self.mode == "stereo" and self.rectify_stereo:
            self.load_stereo_rectification_maps()

        self.publish_exp_config_ = self.create_publisher(
            String, f"{self.output_prefix}/experiment_settings", 1
        )
        self.publish_timestep_ = self.create_publisher(
            Float64, f"{self.output_prefix}/timestep_msg", 1
        )

        if self.mode == "mono":
            self.publish_img_ = self.create_publisher(
                Image, f"{self.output_prefix}/img_msg", qos_profile_sensor_data
            )
        else:
            self.publish_left_img_ = self.create_publisher(
                Image, f"{self.output_prefix}/left_img_msg", qos_profile_sensor_data
            )
            self.publish_right_img_ = self.create_publisher(
                Image, f"{self.output_prefix}/right_img_msg", qos_profile_sensor_data
            )

        self.subscribe_exp_ack_ = self.create_subscription(
            String,
            f"{self.output_prefix}/exp_settings_ack",
            self.ack_callback,
            10,
        )
        self.shutdown_client = self.create_client(Trigger, self.shutdown_service_name)

        self.state_timer = self.create_timer(0.05, self.state_machine_tick)
        self.config_timer = self.create_timer(0.1, self.config_tick)

        self.get_logger().info(
            f"Rosbag adapter started in {self.mode} mode -> {self.output_prefix}"
        )
        self.get_logger().info(
            f"Using direct EuRoC bag read: mono={self.input_image_topic}, "
            f"left={self.left_image_topic}, right={self.right_image_topic}"
        )
        if self.mode == "stereo":
            if self.rectify_stereo:
                self.get_logger().info(
                    f"Stereo stream is rectified before publish using {self.rectify_settings_file}"
                )
            else:
                self.get_logger().info("Stereo stream is forwarded without adapter rectification")
        self.get_logger().info(
            f"Supervised bag path: {self.bag_dir} (playback_rate={self.playback_rate:.2f}x)"
        )

    def load_stereo_rectification_maps(self):
        fs = cv2.FileStorage(str(self.rectify_settings_file), cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise FileNotFoundError(
                f"Could not open rectification settings file: {self.rectify_settings_file}"
            )
        try:
            camera_type = fs.getNode("Camera.type").string()
            if camera_type == "Rectified":
                raise ValueError(
                    "rectify_settings_file must point to the raw stereo settings, not an already rectified file"
                )

            k1 = np.array(
                [
                    [fs.getNode("Camera1.fx").real(), 0.0, fs.getNode("Camera1.cx").real()],
                    [0.0, fs.getNode("Camera1.fy").real(), fs.getNode("Camera1.cy").real()],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            k2 = np.array(
                [
                    [fs.getNode("Camera2.fx").real(), 0.0, fs.getNode("Camera2.cx").real()],
                    [0.0, fs.getNode("Camera2.fy").real(), fs.getNode("Camera2.cy").real()],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            d1 = np.array(
                [
                    fs.getNode("Camera1.k1").real(),
                    fs.getNode("Camera1.k2").real(),
                    fs.getNode("Camera1.p1").real(),
                    fs.getNode("Camera1.p2").real(),
                ],
                dtype=np.float64,
            )
            d2 = np.array(
                [
                    fs.getNode("Camera2.k1").real(),
                    fs.getNode("Camera2.k2").real(),
                    fs.getNode("Camera2.p1").real(),
                    fs.getNode("Camera2.p2").real(),
                ],
                dtype=np.float64,
            )

            width = int(fs.getNode("Camera.width").real())
            height = int(fs.getNode("Camera.height").real())
            image_size = (width, height)

            t_c1_c2 = fs.getNode("Stereo.T_c1_c2").mat()
            if t_c1_c2 is None:
                raise ValueError("Stereo.T_c1_c2 not found in raw stereo settings")
            t_c1_c2 = np.asarray(t_c1_c2, dtype=np.float64)
            if t_c1_c2.shape == (3, 4):
                t_c1_c2 = np.vstack([t_c1_c2, np.array([0.0, 0.0, 0.0, 1.0])])
            t_c2_c1 = np.linalg.inv(t_c1_c2)[:3, :]
            r12 = t_c2_c1[:3, :3]
            t12 = t_c2_c1[:3, 3].reshape(3, 1)

            r1, r2, p1, p2, _, _, _ = cv2.stereoRectify(
                k1,
                d1,
                k2,
                d2,
                image_size,
                r12,
                t12,
                flags=cv2.CALIB_ZERO_DISPARITY,
                alpha=-1,
                newImageSize=image_size,
            )
            self.left_rectify_map_1, self.left_rectify_map_2 = cv2.initUndistortRectifyMap(
                k1, d1, r1, p1[:3, :3], image_size, cv2.CV_32F
            )
            self.right_rectify_map_1, self.right_rectify_map_2 = cv2.initUndistortRectifyMap(
                k2, d2, r2, p2[:3, :3], image_size, cv2.CV_32F
            )
        finally:
            fs.release()

    def rectify_image_message(self, msg: Image, *, is_left: bool) -> Image:
        if not self.rectify_stereo:
            return msg

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if is_left:
            rectified = cv2.remap(image, self.left_rectify_map_1, self.left_rectify_map_2, cv2.INTER_LINEAR)
        else:
            rectified = cv2.remap(image, self.right_rectify_map_1, self.right_rectify_map_2, cv2.INTER_LINEAR)

        rectified_msg = self.bridge.cv2_to_imgmsg(rectified, encoding=msg.encoding)
        rectified_msg.header = msg.header
        return rectified_msg

    def ack_callback(self, msg):
        if msg.data != "ACK" or self.config_acked:
            return

        self.get_logger().info("Config ACK received")
        self.flush_pending_messages()
        self.config_acked = True
        self.config_ack_event.set()
        if self.bag_exit_monotonic is not None:
            self.state = AdapterState.SHUTDOWN_PENDING
        else:
            self.state = AdapterState.STREAMING

    def orbslam_available(self):
        service_ready = self.shutdown_client.wait_for_service(timeout_sec=0.0)
        config_subscribers = self.count_subscribers(f"{self.output_prefix}/experiment_settings")
        ack_publishers = self.count_publishers(f"{self.output_prefix}/exp_settings_ack")
        return service_ready and config_subscribers > 0 and ack_publishers > 0

    def start_bag_reader(self):
        if self.reader_thread is not None:
            return

        self.reader_thread = threading.Thread(
            target=self.bag_reader_main,
            name=f"{self.mode}_bag_reader",
            daemon=True,
        )
        self.reader_thread.start()
        self.get_logger().info(f"Bag start: direct read {self.bag_dir}")

    def config_tick(self):
        if self.state != AdapterState.HANDSHAKE_ACTIVE or self.config_acked:
            return

        msg = String()
        msg.data = self.experiment_settings_payload
        self.publish_exp_config_.publish(msg)
        self.config_send_count += 1

        now = time.monotonic()
        if self.last_config_log_time is None or (now - self.last_config_log_time) >= 10.0:
            self.last_config_log_time = now
            subs = self.count_subscribers(f"{self.output_prefix}/experiment_settings")
            ack_pubs = self.count_publishers(f"{self.output_prefix}/exp_settings_ack")
            self.get_logger().info(
                f"Handshake: sent {self.config_send_count} config msgs; "
                f"experiment_settings subscribers={subs}, exp_settings_ack publishers={ack_pubs}"
            )

    # Number of consecutive orbslam_available() True-ticks required before starting bag.
    # Each tick is 50 ms (state_timer), so 3 ticks = 150 ms minimum confirmed connection.
    ORBSLAM_READY_REQUIRED = 3

    def state_machine_tick(self):
        if self.state == AdapterState.WAIT_FOR_ORBSLAM:
            if self.orbslam_available():
                self.orbslam_ready_count += 1
                if self.orbslam_ready_count >= self.ORBSLAM_READY_REQUIRED:
                    self.get_logger().info(
                        f"ORB-SLAM wrapper confirmed available "
                        f"({self.ORBSLAM_READY_REQUIRED} consecutive checks)"
                    )
                    self.start_bag_reader()
                    self.state = AdapterState.WAIT_FOR_FIRST_FRAME
            else:
                if self.orbslam_ready_count > 0:
                    self.get_logger().warning(
                        f"ORB-SLAM availability check dropped to False "
                        f"(was {self.orbslam_ready_count}/{self.ORBSLAM_READY_REQUIRED}), resetting"
                    )
                self.orbslam_ready_count = 0
            return

        if self.state == AdapterState.SHUTDOWN_PENDING:
            self.process_shutdown_state()

    def process_shutdown_state(self):
        if self.shutdown_future is not None:
            if self.shutdown_future.done():
                try:
                    response = self.shutdown_future.result()
                except Exception as exc:  # pragma: no cover - defensive
                    self.get_logger().error(f"Shutdown ACK failed: {exc}")
                else:
                    if response.success:
                        self.get_logger().info(f"Shutdown ACK received: {response.message}")
                    else:
                        self.get_logger().error(
                            f"Shutdown ACK reported failure: {response.message}"
                        )
                self.state = AdapterState.DONE
                rclpy.shutdown()
                return

            elapsed = time.monotonic() - self.shutdown_request_monotonic
            if elapsed >= self.shutdown_timeout_sec:
                self.get_logger().error("Shutdown ACK timed out while waiting for ORB-SLAM")
                self.state = AdapterState.DONE
                rclpy.shutdown()
            return

        if self.bag_exit_monotonic is None:
            return

        elapsed_since_bag_exit = time.monotonic() - self.bag_exit_monotonic
        if elapsed_since_bag_exit < self.shutdown_drain_sec:
            return

        self.send_shutdown_request()

    def send_shutdown_request(self):
        if self.shutdown_request_sent:
            return

        request = Trigger.Request()
        self.shutdown_future = self.shutdown_client.call_async(request)
        self.shutdown_request_sent = True
        self.shutdown_request_monotonic = time.monotonic()
        self.get_logger().info("Shutdown request sent")

    def stamp_to_seconds(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def publish_timestamp(self, stamp):
        timestep_msg = Float64()
        timestep_msg.data = self.stamp_to_seconds(stamp)
        self.publish_timestep_.publish(timestep_msg)

    def maybe_begin_handshake(self):
        if self.first_frame_seen:
            return

        self.first_frame_seen = True
        self.state = AdapterState.HANDSHAKE_ACTIVE
        if self.mode == "mono":
            self.get_logger().info("First frame seen, starting config handshake")
        else:
            self.get_logger().info("First stereo pair seen, starting config handshake")

    def flush_pending_messages(self):
        if self.mode == "mono":
            if self.pending_mono_msg is not None:
                self.publish_mono_message(self.pending_mono_msg)
                self.pending_mono_msg = None
            return

        if self.pending_left_msg is not None and self.pending_right_msg is not None:
            self.publish_stereo_messages(self.pending_left_msg, self.pending_right_msg)
            self.pending_left_msg = None
            self.pending_right_msg = None

    def publish_mono_message(self, msg):
        with self.publish_lock:
            self.last_published_stamp_sec = self.stamp_to_seconds(msg.header.stamp)
            self.publish_timestamp(msg.header.stamp)
            self.publish_img_.publish(msg)

    def publish_stereo_messages(self, left_msg, right_msg):
        with self.publish_lock:
            self.last_published_stamp_sec = self.stamp_to_seconds(left_msg.header.stamp)
            self.publish_timestamp(left_msg.header.stamp)
            self.publish_left_img_.publish(left_msg)
            self.publish_right_img_.publish(right_msg)

    def handle_mono_message(self, msg):
        if self.state == AdapterState.DONE:
            return

        if self.state == AdapterState.WAIT_FOR_FIRST_FRAME:
            self.pending_mono_msg = msg
            self.maybe_begin_handshake()
            return

        if self.state == AdapterState.HANDSHAKE_ACTIVE and not self.config_acked:
            self.pending_mono_msg = msg
            return

        if self.state in {AdapterState.STREAMING, AdapterState.SHUTDOWN_PENDING}:
            self.publish_mono_message(msg)

    def handle_stereo_pair(self, left_msg, right_msg):
        if self.state == AdapterState.DONE:
            return

        if self.state == AdapterState.WAIT_FOR_FIRST_FRAME:
            self.pending_left_msg = left_msg
            self.pending_right_msg = right_msg
            self.maybe_begin_handshake()
            return

        if self.state == AdapterState.HANDSHAKE_ACTIVE and not self.config_acked:
            self.pending_left_msg = left_msg
            self.pending_right_msg = right_msg
            return

        if self.state in {AdapterState.STREAMING, AdapterState.SHUTDOWN_PENDING}:
            self.publish_stereo_messages(left_msg, right_msg)

    def pace_to_next_message(self, previous_bag_time_ns, next_bag_time_ns, previous_wall_time):
        if previous_bag_time_ns is None or previous_wall_time is None:
            return time.monotonic()

        bag_delta_sec = max(0.0, (next_bag_time_ns - previous_bag_time_ns) / 1e9)
        desired_delta_sec = bag_delta_sec / self.playback_rate
        elapsed_wall_sec = time.monotonic() - previous_wall_time
        sleep_duration = desired_delta_sec - elapsed_wall_sec
        if sleep_duration > 0.0 and not self.stop_event.is_set():
            time.sleep(sleep_duration)

        return time.monotonic()

    def pace_to_next_header_stamp(self, previous_stamp_sec, next_stamp_sec):
        if previous_stamp_sec is None:
            self._pace_wall_anchor = time.monotonic()
            return self._pace_wall_anchor

        delta_sec = max(0.0, next_stamp_sec - previous_stamp_sec) / self.playback_rate
        elapsed_wall_sec = time.monotonic() - self._pace_wall_anchor
        sleep_duration = delta_sec - elapsed_wall_sec
        if sleep_duration > 0.0 and not self.stop_event.is_set():
            time.sleep(sleep_duration)

        self._pace_wall_anchor = time.monotonic()
        return self._pace_wall_anchor

    def wait_for_config_ack(self):
        deadline = time.monotonic() + self.handshake_timeout_sec
        while not self.config_ack_event.wait(timeout=0.1):
            if self.stop_event.is_set():
                return
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"Handshake timed out after {self.handshake_timeout_sec:.0f}s "
                    f"({self.config_send_count} config msgs sent). "
                    "ORB-SLAM3 C++ node did not ACK — aborting bag reader."
                )
                self.stop_event.set()
                return

    def open_bag_reader(self):
        reader = rosbag2_py.SequentialReader()
        if self.bag_dir.is_file():
            bag_path = self.bag_dir
        else:
            bag_files = sorted(
                path
                for path in self.bag_dir.iterdir()
                if path.is_file() and path.suffix.lower() in SUPPORTED_BAG_SUFFIXES
            )
            if not bag_files:
                suffixes = ", ".join(SUPPORTED_BAG_SUFFIXES)
                raise FileNotFoundError(
                    f"No supported bag file ({suffixes}) found in {self.bag_dir}"
                )
            # When the bag directory is on a read-only filesystem (e.g. a
            # Docker bind mount), opening the directory URI can trigger a
            # metadata write attempt. Opening the actual storage file avoids
            # that path for both sqlite3 and MCAP bags.
            bag_path = bag_files[0]

        storage_id = "mcap" if bag_path.suffix.lower() == ".mcap" else "sqlite3"
        storage_options = rosbag2_py.StorageOptions(
            uri=str(bag_path),
            storage_id=storage_id,
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        reader.open(storage_options, converter_options)
        return reader

    def bag_reader_main(self):
        previous_stamp_sec = None

        try:
            reader = self.open_bag_reader()
            pending_left = {}
            pending_right = {}

            while reader.has_next() and not self.stop_event.is_set():
                topic_name, serialized_data, _bag_time_ns = reader.read_next()

                if self.mode == "mono":
                    if topic_name != self.input_image_topic:
                        time.sleep(0)  # yield GIL so spin thread can fire timers
                        continue

                    image_msg = deserialize_message(serialized_data, Image)
                    if not self.config_acked:
                        if not self.first_frame_seen:
                            self.pending_mono_msg = image_msg
                            self.maybe_begin_handshake()
                        self.wait_for_config_ack()
                        if self.stop_event.is_set() or self.state == AdapterState.DONE:
                            break
                        previous_stamp_sec = self.last_published_stamp_sec
                        continue

                    stamp_sec = self.stamp_to_seconds(image_msg.header.stamp)
                    self.pace_to_next_header_stamp(previous_stamp_sec, stamp_sec)
                    previous_stamp_sec = stamp_sec
                    self.publish_mono_message(image_msg)
                    continue

                if topic_name not in {self.left_image_topic, self.right_image_topic}:
                    time.sleep(0)  # yield GIL so spin thread can fire timers
                    continue

                image_msg = deserialize_message(serialized_data, Image)
                stamp_key = (
                    int(image_msg.header.stamp.sec),
                    int(image_msg.header.stamp.nanosec),
                )

                if topic_name == self.left_image_topic:
                    pending_left[stamp_key] = image_msg
                else:
                    pending_right[stamp_key] = image_msg

                if stamp_key not in pending_left or stamp_key not in pending_right:
                    continue

                left_msg = self.rectify_image_message(pending_left.pop(stamp_key), is_left=True)
                right_msg = self.rectify_image_message(pending_right.pop(stamp_key), is_left=False)

                if not self.config_acked:
                    if not self.first_frame_seen:
                        self.pending_left_msg = left_msg
                        self.pending_right_msg = right_msg
                        self.maybe_begin_handshake()
                    self.wait_for_config_ack()
                    if self.stop_event.is_set() or self.state == AdapterState.DONE:
                        break
                    previous_stamp_sec = self.last_published_stamp_sec
                    continue

                stamp_sec = self.stamp_to_seconds(left_msg.header.stamp)
                self.pace_to_next_header_stamp(previous_stamp_sec, stamp_sec)
                previous_stamp_sec = stamp_sec
                self.publish_stereo_messages(left_msg, right_msg)

            self.bag_exit_code = 0
        except Exception as exc:  # pragma: no cover - defensive
            self.bag_exit_code = 1
            self.get_logger().error(f"Bag reader failed: {exc}")
        finally:
            if self.bag_exit_monotonic is None:
                self.bag_exit_monotonic = time.monotonic()
                self.state = AdapterState.SHUTDOWN_PENDING
                self.get_logger().info(f"Bag exited with code {self.bag_exit_code}")

    def terminate_bag_reader(self):
        self.stop_event.set()
        if self.reader_thread is not None and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = RosbagAdapter()

    # Use MultiThreadedExecutor so the ROS spin loop (timers, callbacks) runs in
    # its own OS thread and can compete for the GIL independently from the bag
    # reader thread. With rclpy.spin() (SingleThreadedExecutor), the bag reader's
    # tight rosbag2 read loop holds the GIL and starves the config_tick timer.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.terminate_bag_reader()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
