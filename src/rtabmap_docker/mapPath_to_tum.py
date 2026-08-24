#!/usr/bin/env python3

import argparse
import os
import sys
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import MapData
from rtabmap_msgs.srv import GetMap2


class TrajectoryToTum(Node):
    def __init__(
        self,
        topic,
        output_path,
        discovery_timeout,
        msg_timeout,
        idle_timeout,
        qos_mode,
        fsync_each_write,
    ):
        super().__init__("map_path_to_tum")

        self.topic = topic
        self.output_path = output_path
        self.idle_timeout = idle_timeout
        self.fsync_each_write = fsync_each_write
        self.mode = None
        self.count = 0
        self.last_update_time = None
        self.warned_path_stamps = False

        # Persistent timestamp cache so nodes that age out of STM into LTM
        # (and therefore disappear from msg.nodes) are not silently dropped.
        self._stamp_cache: dict = {}
        # Track which node IDs have already been appended to the output file so
        # we never emit the same node twice and the file only ever grows.
        self._written_ids: set = set()
        # Last timestamp written — used to keep the appended file monotonic.
        self._last_written_stamp: float = None  # type: ignore[assignment]

        # Start each recorder session with a clean (empty) file so that
        # append mode begins fresh rather than accumulating across sessions.
        try:
            open(output_path, "w").close()
        except OSError:
            pass

        self.get_logger().info(f"Checking if topic exists: {topic}")

        deadline = time.time() + discovery_timeout
        found_types = []
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            topics = dict(self.get_topic_names_and_types())
            if topic in topics:
                found_types = topics[topic]
                break

        if not found_types:
            self.get_logger().error(f"Topic does not exist: {topic}")
            self.print_trajectory_like_topics()
            raise RuntimeError(f"Required topic missing: {topic}")

        if "rtabmap_msgs/msg/MapData" in found_types:
            self.mode = "mapdata"
            msg_type = MapData
        elif "nav_msgs/msg/Path" in found_types:
            self.mode = "path"
            msg_type = Path
        else:
            self.get_logger().error(f"Unsupported topic type(s): {found_types}")
            raise RuntimeError(f"Topic {topic} must be rtabmap_msgs/msg/MapData or nav_msgs/msg/Path")

        if qos_mode == "best_effort":
            reliability = ReliabilityPolicy.BEST_EFFORT
        elif qos_mode == "reliable":
            reliability = ReliabilityPolicy.RELIABLE
        else:
            raise RuntimeError(f"Unknown QoS mode: {qos_mode}")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(msg_type, topic, self.callback, qos)

        self.deadline_timer = self.create_timer(msg_timeout, self.check_first_message_timeout)
        self.idle_timer = None
        if idle_timeout > 0.0:
            self.idle_timer = self.create_timer(0.5, self.check_idle_timeout)

        self.get_logger().info(f"Recording trajectory topic: {topic}")
        self.get_logger().info(f"Detected mode: {self.mode}")
        self.get_logger().info(f"QoS reliability: {qos_mode}")
        self.get_logger().info(f"Writing TUM trajectory to: {output_path}")
        self.get_logger().info(f"Idle timeout: {idle_timeout:.1f}s (0 disables auto-exit)")
        if self.mode == "mapdata":
            self.get_logger().info("Using optimized graph poses from /rtabmap/mapData with node timestamps.")
        else:
            self.get_logger().warn(
                "Using nav_msgs/Path fallback. If pose stamps are all identical, tiny synthetic offsets will be used."
            )

    def print_trajectory_like_topics(self):
        self.get_logger().error("Available path/graph-like topics:")
        for name, types in self.get_topic_names_and_types():
            lowered = name.lower()
            if "path" in lowered or "map" in lowered or "graph" in lowered or "odom" in lowered:
                self.get_logger().error(f"  {name}: {types}")

    def check_first_message_timeout(self):
        if self.count == 0:
            self.get_logger().error(f"No compatible messages received from {self.topic}.")
            self.get_logger().error("Try a different QoS or verify the topic with:")
            self.get_logger().error(f"  ros2 topic info -v {self.topic}")
            raise RuntimeError(f"No compatible messages received from {self.topic}")
        self.deadline_timer.cancel()

    def check_idle_timeout(self):
        if self.count == 0 or self.last_update_time is None:
            return
        if time.time() - self.last_update_time >= self.idle_timeout:
            self.get_logger().info("No trajectory updates received before idle timeout. Exiting.")
            raise SystemExit

    def callback(self, msg):
        if self.mode == "mapdata":
            records, new_ids = self.records_from_mapdata(msg)
        else:
            records = self.records_from_path(msg)
            new_ids = set()

        if not records:
            self.get_logger().warn("Received message but no new trajectory poses to append.")
            return

        self.write_records(records)
        self._written_ids.update(new_ids)
        self.count += 1
        self.last_update_time = time.time()
        if records:
            self._last_written_stamp = records[-1][0]

        first_t = records[0][0]
        last_t = records[-1][0]
        self.get_logger().info(
            f"Appended {len(records)} new poses "
            f"(total in file: {len(self._written_ids)}) "
            f"time range [{first_t:.9f}, {last_t:.9f}]"
        )

    def records_from_mapdata(self, msg):
        # Persist timestamps from nodes currently in memory (STM + recently
        # accessed LTM).  Once a node ages fully into LTM it disappears from
        # msg.nodes, but its stamp stays in the cache so we never lose it.
        for node in msg.nodes:
            if node.stamp > 0.0:
                self._stamp_cache[node.id] = float(node.stamp)

        raw_new = []
        missing = 0
        for node_id, pose in zip(msg.graph.poses_id, msg.graph.poses):
            if node_id in self._written_ids:
                continue  # already appended in a previous callback
            stamp = self._stamp_cache.get(node_id)
            if stamp is None:
                missing += 1
                continue
            raw_new.append((stamp, int(node_id), pose))

        if missing:
            self.get_logger().warn(
                f"Skipped {missing} graph poses without cached timestamps "
                f"(nodes moved to LTM before first being seen in msg.nodes)."
            )

        raw_new.sort(key=lambda item: (item[0], item[1]))
        new_ids = {item[1] for item in raw_new}

        # Ensure the first record of this batch sits strictly after the last
        # timestamp already written to the file.
        start_after = self._last_written_stamp
        records = self.make_monotonic(
            [(stamp, pose.position, pose.orientation) for stamp, _, pose in raw_new],
            start_after=start_after,
        )
        return records, new_ids

    def records_from_path(self, msg):
        if not msg.poses:
            return []

        pose_stamps = [pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9 for pose in msg.poses]
        unique_stamps = len(set(pose_stamps))

        if unique_stamps <= 1:
            if not self.warned_path_stamps:
                self.get_logger().warn(
                    "All poses in nav_msgs/Path share the same timestamp. "
                    "Using tiny synthetic offsets so the output stays valid for trajectory tools."
                )
                self.warned_path_stamps = True
            base = pose_stamps[0] if pose_stamps[0] > 0.0 else (
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            )
            records = []
            for i, pose in enumerate(msg.poses):
                records.append((base + i * 1e-6, pose.pose.position, pose.pose.orientation))
            return records

        raw_records = []
        for pose in msg.poses:
            stamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
            raw_records.append((stamp, pose.pose.position, pose.pose.orientation))

        raw_records.sort(key=lambda item: item[0])
        return self.make_monotonic(raw_records)

    def make_monotonic(self, records, start_after=None):
        adjusted = []
        last_t = start_after  # honours cross-batch monotonicity when appending
        fixed = 0
        for stamp, position, orientation in records:
            t = stamp
            if last_t is not None and t <= last_t:
                t = last_t + 1e-6
                fixed += 1
            adjusted.append((t, position, orientation))
            last_t = t

        if fixed:
            self.get_logger().warn(f"Adjusted {fixed} timestamps to keep them strictly increasing.")

        return adjusted

    def write_records(self, records):
        # Append mode: the file only grows during a session.
        # The file was truncated to zero at __init__ time (fresh session start).
        with open(self.output_path, "a", buffering=1) as f:
            for t, p, q in records:
                f.write(
                    f"{t:.9f} "
                    f"{p.x:.9f} {p.y:.9f} {p.z:.9f} "
                    f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"
                )
            f.flush()
            if self.fsync_each_write:
                os.fsync(f.fileno())


class ServiceTrajectoryToTum(Node):
    def __init__(self, service_name, output_path, timeout, idle_timeout, poll_period, fsync_each_write):
        super().__init__("map_path_to_tum_service")
        self.service_name = service_name
        self.output_path = output_path
        self.timeout = timeout
        self.idle_timeout = idle_timeout
        self.poll_period = poll_period
        self.fsync_each_write = fsync_each_write
        self.client = None

    def resolve_service_name(self):
        if self.service_name not in ("", "auto"):
            return self.service_name

        deadline = time.time() + self.timeout
        candidates = []
        while time.time() < deadline and rclpy.ok():
            for name, types in self.get_service_names_and_types():
                if "rtabmap_msgs/srv/GetMap2" in types and name.endswith("/get_map_data2"):
                    candidates.append(name)
            if candidates:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        if not candidates:
            return None

        # Prefer the nested service first as that's what rtabmap often exposes.
        candidates = sorted(set(candidates), key=lambda n: (0 if n.endswith("/rtabmap/get_map_data2") else 1, n))
        return candidates[0]

    def log_service_debug(self):
        rtabmap_services = []
        for name, types in self.get_service_names_and_types():
            if "rtabmap" in name:
                rtabmap_services.append(f"{name}: {types}")
        if rtabmap_services:
            self.get_logger().error("Visible RTAB-Map-related services:")
            for line in sorted(rtabmap_services):
                self.get_logger().error(f"  {line}")
        else:
            self.get_logger().error("No RTAB-Map-related services are visible.")

        try:
            names = self.get_node_names()
            rtabmap_nodes = [name for name in names if "rtabmap" in name]
            if rtabmap_nodes:
                self.get_logger().error(f"Visible RTAB-Map-related nodes: {sorted(rtabmap_nodes)}")
            else:
                self.get_logger().error("No RTAB-Map-related nodes are visible.")
        except Exception:
            pass

    def run_export(self):
        resolved_name = self.resolve_service_name()
        if resolved_name is None:
            self.get_logger().error(
                "Could not find any GetMap2 service ending with /get_map_data2. "
                "Make sure the /rtabmap node is running before exporting."
            )
            self.log_service_debug()
            raise RuntimeError("Required RTAB-Map GetMap2 service is not available.")

        self.service_name = resolved_name
        self.client = self.create_client(GetMap2, self.service_name)
        self.get_logger().info(f"Waiting for service: {self.service_name}")
        if not self.client.wait_for_service(timeout_sec=self.timeout):
            self.get_logger().error(f"Service not available: {self.service_name}")
            self.log_service_debug()
            raise RuntimeError(f"Required service missing: {self.service_name}")

        req = GetMap2.Request()
        req.global_map = True
        req.optimized = True
        req.with_images = False
        req.with_scans = False
        req.with_user_data = False
        req.with_grids = False
        req.with_words = False
        req.with_global_descriptors = False

        self.get_logger().info("Polling optimized RTAB-Map graph with node timestamps.")
        self.get_logger().info(
            f"Service mode will stop after {self.idle_timeout:.1f}s without trajectory growth "
            f"(set --idle-timeout 0 for a single snapshot)."
        )

        last_key = None
        last_change_time = None
        snapshots = 0

        while rclpy.ok():
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
            response = future.result()
            if response is None:
                raise RuntimeError("No response received from get_map_data2 service.")

            records = self.records_from_mapdata(response.data)
            if not records:
                raise RuntimeError("Service returned no trajectory poses with valid timestamps.")

            key = (len(records), records[-1][0])
            now = time.time()
            if key != last_key:
                self.write_records(records)
                snapshots += 1
                last_key = key
                last_change_time = now
                first_t = records[0][0]
                last_t = records[-1][0]
                self.get_logger().info(
                    f"Wrote trajectory snapshot #{snapshots}: {len(records)} poses, "
                    f"time range [{first_t:.9f}, {last_t:.9f}]"
                )
                if self.idle_timeout <= 0.0:
                    self.get_logger().info("Single-snapshot mode completed.")
                    break
            elif self.idle_timeout <= 0.0:
                break
            elif last_change_time is not None and now - last_change_time >= self.idle_timeout:
                if self.rosbag_player_alive():
                    self.get_logger().warn(
                        "Trajectory has not grown recently, but /rosbag2_player is still running. Waiting."
                    )
                else:
                    self.get_logger().info(
                        "Trajectory stopped growing and /rosbag2_player is no longer running. Exiting."
                    )
                    break

            if self.poll_period > 0.0:
                end = time.time() + self.poll_period
                while rclpy.ok() and time.time() < end:
                    rclpy.spin_once(self, timeout_sec=0.1)

    def rosbag_player_alive(self):
        try:
            for name, namespace in self.get_node_names_and_namespaces():
                if name == "rosbag2_player":
                    return True
        except Exception:
            pass
        return False

    def records_from_mapdata(self, msg):
        stamp_by_id = {}
        for node in msg.nodes:
            if node.stamp > 0.0:
                stamp_by_id[node.id] = float(node.stamp)

        raw_records = []
        missing = 0
        for node_id, pose in zip(msg.graph.poses_id, msg.graph.poses):
            stamp = stamp_by_id.get(node_id)
            if stamp is None:
                missing += 1
                continue
            raw_records.append((stamp, int(node_id), pose))

        if missing:
            self.get_logger().warn(f"Skipped {missing} graph poses without node stamps.")

        raw_records.sort(key=lambda item: (item[0], item[1]))
        return self.make_monotonic(
            [(stamp, pose.position, pose.orientation) for stamp, _, pose in raw_records]
        )

    def make_monotonic(self, records):
        adjusted = []
        last_t = None
        fixed = 0
        for stamp, position, orientation in records:
            t = stamp
            if last_t is not None and t <= last_t:
                t = last_t + 1e-6
                fixed += 1
            adjusted.append((t, position, orientation))
            last_t = t

        if fixed:
            self.get_logger().warn(f"Adjusted {fixed} timestamps to keep them strictly increasing.")

        return adjusted

    def write_records(self, records):
        tmp_path = self.output_path + ".tmp"
        with open(tmp_path, "w", buffering=1) as f:
            for t, p, q in records:
                f.write(
                    f"{t:.9f} "
                    f"{p.x:.9f} {p.y:.9f} {p.z:.9f} "
                    f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"
                )
            f.flush()
            if self.fsync_each_write:
                os.fsync(f.fileno())
        os.replace(tmp_path, self.output_path)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Export RTAB-Map's optimized trajectory to TUM format. "
            "By default, it polls /rtabmap/rtabmap/get_map_data2 because "
            "/rtabmap/mapPath reuses one timestamp for the whole path."
        )
    )
    parser.add_argument("--source", choices=["service", "topic"], default="service")
    parser.add_argument("--topic", default="/rtabmap/mapData")
    parser.add_argument("--service-name", default="auto")
    parser.add_argument("--out", default="/workspace/rtabmap_mapPath.tum")
    parser.add_argument("--discovery-timeout", type=float, default=5.0)
    parser.add_argument("--msg-timeout", type=float, default=5.0)
    parser.add_argument("--idle-timeout", type=float, default=5.0)
    parser.add_argument("--poll-period", type=float, default=1.0)
    parser.add_argument("--qos", choices=["best_effort", "reliable"], default="reliable")
    parser.add_argument("--no-fsync", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = None
    try:
        if args.source == "service":
            node = ServiceTrajectoryToTum(
                service_name=args.service_name,
                output_path=args.out,
                timeout=args.discovery_timeout,
                idle_timeout=args.idle_timeout,
                poll_period=args.poll_period,
                fsync_each_write=not args.no_fsync,
            )
            node.run_export()
        else:
            node = TrajectoryToTum(
                topic=args.topic,
                output_path=args.out,
                discovery_timeout=args.discovery_timeout,
                msg_timeout=args.msg_timeout,
                idle_timeout=args.idle_timeout,
                qos_mode=args.qos,
                fsync_each_write=not args.no_fsync,
            )
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    except Exception as e:
        print(f"[FATAL] {e}", file=sys.stderr)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    if node is not None:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
