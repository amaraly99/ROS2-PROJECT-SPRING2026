"""RTAB-Map stereo sidecar for the LIVE HIL rig (created 2026-09-08).

Mirrors euroc_offline_f2m.launch.py (the F2M stereo configuration this project already
benchmarked on EuRoC, Table 2 of Labbe & Michaud 2024) with three live-HIL changes:

  * inputs are the HIL camera bridges (/ovcam/image_raw, /ovcam/right/image_raw) -- ideal
    rendered pinhole pair, already rectified, identical stamps per pair -- so no
    preprocessor/rectifier; CameraInfo comes from hil_stereo_bridge.py;
  * frame_id is the LEFT image frame ("camera", ovcam_bridge's default) so no TF tree is
    needed between a body frame and the optical frame; the baseline comes from the right
    CameraInfo Tx, not TF;
  * wall clock (no use_sim_time); always_process_most_recent_frame for a live stream.

Run from the stack config as PID 1 (docker stop -> SIGTERM -> launch tears children down):
    bash -c "exec ros2 launch /workspace/src/rtabmap_docker/hil_stereo.launch.py"
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    common_params = {
        "frame_id":               "camera",
        "subscribe_stereo":       True,
        "subscribe_rgb":          False,
        "subscribe_depth":        False,
        "subscribe_odom_info":    True,
        "wait_imu_to_init":       False,
        "qos":                    2,
        "qos_image":              2,
        "qos_camera_info":        2,
        "qos_odom":               2,
        # Both eyes and both CameraInfo carry the identical MATLAB stamp per pair.
        "approx_sync":            False,
        "odom_sensor_sync":       True,
        "topic_queue_size":       100,
        "sync_queue_size":        100,

        # ── Detector (GFTT) -- Table 2 defaults ────────────────────────────
        "GFTT/MinDistance":       "3",
        "GFTT/QualityLevel":      "0.001",
        "Kp/MaxFeatures":         "500",

        # ── F2M odometry ───────────────────────────────────────────────────
        "Odom/KeyFrameThr":       "0.3",
        "OdomF2M/MaxSize":        "2000",

        # ── Visual matching (loop closure + odometry recovery) ─────────────
        "Vis/MaxFeatures":        "1000",
        "Vis/MinInliers":         "20",
        "Vis/CorNNDR":            "0.6",

        # ── Memory management disabled (paper setting) ─────────────────────
        "Rtabmap/TimeThr":        "0",
        "Rtabmap/MemoryThr":      "0",
        "Mem/STMSize":            "30",

        # ── Graph / SLAM node ──────────────────────────────────────────────
        "Rtabmap/DetectionRate":           "2",
        "Rtabmap/CreateIntermediateNodes": "true",
        "RGBD/CreateOccupancyGrid":        "false",
        "RGBD/LinearUpdate":               "0",
        "RGBD/AngularUpdate":              "0",
        "RGBD/OptimizeMaxError":           "1",
    }

    remappings = [
        ("left/image_rect",   "/ovcam/image_raw"),
        ("left/camera_info",  "/ovcam/camera_info"),
        ("right/image_rect",  "/ovcam/right/image_raw"),
        ("right/camera_info", "/ovcam/right/camera_info"),
    ]

    return LaunchDescription([
        ExecuteProcess(
            cmd=["python3", "/workspace/src/rtabmap_docker/hil_stereo_bridge.py"],
            output="screen",
        ),
        Node(
            package="rtabmap_odom",
            executable="stereo_odometry",
            output="screen",
            parameters=[common_params, {
                # Live stream: keep up with the camera rather than queueing behind it.
                "always_process_most_recent_frame": True,
                "publish_null_when_lost": False,
            }],
            remappings=remappings,
            arguments=["-d"],
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            output="screen",
            parameters=[common_params],
            remappings=remappings,
            arguments=["-d"],
        ),
    ])
