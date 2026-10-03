"""RTAB-Map RGB-D sidecar for the LIVE HIL rig (created 2026-09-27).

RGB-D twin of hil_stereo.launch.py -- same Table-2 parameters, same live-HIL choices,
only the sensor changes:

  * inputs: the left HIL camera (/ovcam/image_raw, via ovcam_bridge on the MATLAB source
    stamp -- run_stack_hil_rgbd.sh / hil_simulation_rgbd.launch.py) plus depth straight
    from the RGB-D Simulink model (/sim/camera/depth/image_raw, 16UC1 millimetres,
    registered to the left camera, same stamp). CameraInfo comes from hil_rgbd_bridge.py;
  * frame_id is the LEFT image frame ("camera") -- the same optical-frame shortcut the
    stereo arm uses, kept on purpose for milestone 1 so RGB-D and stereo are configured
    alike. (On TUM fr1_desk, an x-forward base_link + static TF tracked better; switch
    both arms together after the milestone.);
  * wall clock (no use_sim_time); always_process_most_recent_frame for a live stream.

Launch arguments (defaults = the live HIL setup; overrides exist for offline wiring tests):
  frame_id, depth_topic, fx, fy, cx, cy, use_sim_time

Run from the stack config as PID 1 (docker stop -> SIGTERM -> launch tears children down):
    bash -c "exec ros2 launch /workspace/src/rtabmap_docker/hil_rgbd.launch.py"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    frame_id = LaunchConfiguration("frame_id")
    depth_topic = LaunchConfiguration("depth_topic")
    rgb_topic = LaunchConfiguration("rgb_topic")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    common_params = {
        "frame_id":               frame_id,
        "use_sim_time":           use_sim_time,
        "subscribe_stereo":       False,
        "subscribe_rgb":          False,
        "subscribe_depth":        True,
        "subscribe_odom_info":    True,
        "wait_imu_to_init":       False,
        "qos":                    2,
        "qos_image":              2,
        "qos_camera_info":        2,
        "qos_odom":               2,
        # Left image and depth carry the identical MATLAB stamp per frame.
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
        ("rgb/image",       rgb_topic),
        ("rgb/camera_info", "/ovcam/camera_info"),
        ("depth/image",     depth_topic),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("frame_id", default_value="camera",
                              description="robot frame; 'camera' = left image frame (optical shortcut, as stereo)"),
        DeclareLaunchArgument("depth_topic", default_value="/sim/camera/depth/image_raw"),
        # rgb_topic: default = the gray /ovcam path (unchanged). /sim/camera/image_raw feeds
        # MATLAB's full-res bgr8 color straight to RTAB-Map (same stamp as depth).
        DeclareLaunchArgument("rgb_topic", default_value="/ovcam/image_raw"),
        DeclareLaunchArgument("fx", default_value="554.0"),
        DeclareLaunchArgument("fy", default_value="554.0"),
        DeclareLaunchArgument("cx", default_value="320.0"),
        DeclareLaunchArgument("cy", default_value="240.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        ExecuteProcess(
            cmd=["python3", "/workspace/src/rtabmap_docker/hil_rgbd_bridge.py", "--ros-args",
                 "-p", ["fx:=", LaunchConfiguration("fx")],
                 "-p", ["fy:=", LaunchConfiguration("fy")],
                 "-p", ["cx:=", LaunchConfiguration("cx")],
                 "-p", ["cy:=", LaunchConfiguration("cy")],
                 "-p", ["left_image_topic:=", rgb_topic],
                 "-p", ["use_sim_time:=", LaunchConfiguration("use_sim_time")]],
            output="screen",
        ),
        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
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
