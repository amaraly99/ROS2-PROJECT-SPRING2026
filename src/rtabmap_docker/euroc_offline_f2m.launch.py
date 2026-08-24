from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, SetParameter
import os


def generate_launch_description():
    # ── Parameters from Table 2 of Labbé & Michaud 2024 (arXiv:2403.06341) ──────
    # Target: ATE ≈ 1.7 cm on MH_01_easy (F2M stereo odometry).
    #
    # The two most impactful settings versus a naïve configuration:
    #   Rtabmap/TimeThr   = 0  →  no frame-skipping; every odom frame is processed
    #   Rtabmap/MemoryThr = 0  →  no LTM eviction; full trajectory stays in WM
    #                             so loop closure can fire anywhere in the map.
    common_params = {
        "frame_id":               "base_link",
        "subscribe_stereo":       True,
        "subscribe_rgb":          False,
        "subscribe_depth":        False,    # explicit — suppresses the auto-detect warning
        "subscribe_odom_info":    True,
        "wait_imu_to_init":       False,
        "qos":                    2,
        "qos_image":              2,
        "qos_camera_info":        2,
        "qos_odom":               2,
        # The custom preprocessor republishes left/right/image_info with a
        # shared canonical timestamp, so we can use exact sync again without
        # cross-pairing adjacent stereo frames.
        "approx_sync":            False,
        # Keep SLAM synchronized with valid odometry so image frames from brief
        # VO dropouts are skipped instead of producing RTAB-Map process errors.
        "odom_sensor_sync":       True,
        "topic_queue_size":       100,
        "sync_queue_size":        100,

        # ── Detector (GFTT) ───────────────────────────────────────────────────
        "GFTT/MinDistance":       "3",      # Table 2 default
        "GFTT/QualityLevel":      "0.001",  # Table 2 default
        "Kp/MaxFeatures":         "500",    # Table 2 default

        # ── F2M odometry ──────────────────────────────────────────────────────
        "Odom/KeyFrameThr":       "0.3",    # Table 2 — fraction of matched features
        "OdomF2M/MaxSize":        "2000",   # Table 2 — local feature-map capacity

        # ── Visual matching (loop closure + odometry) ─────────────────────────
        # Vis/MaxFeatures and Vis/MinInliers are shared by BOTH loop closure AND
        # the odometry's no-guess recovery path — do not lower MinInliers below
        # the paper value of 20, as stereo_odometry also uses it.
        "Vis/MaxFeatures":        "1000",   # Table 2 default
        "Vis/MinInliers":         "20",     # Table 2 default — also used by stereo_odometry
        "Vis/CorNNDR":            "0.6",    # Table 2 default

        # ── Memory management: DISABLED for offline evaluation ─────────────────
        # Paper: "memory management has been disabled (TimeThr and MemoryThr = 0)"
        "Rtabmap/TimeThr":        "0",      # no frame-budget cap
        "Rtabmap/MemoryThr":      "0",      # no LTM eviction — full WM for loop closure
        "Mem/STMSize":            "30",     # Table 2 default

        # ── Graph / SLAM node ─────────────────────────────────────────────────
        "Rtabmap/DetectionRate":           "2",     # Table 2 — new SLAM nodes at 2 Hz
        "Rtabmap/CreateIntermediateNodes": "true",  # store every odom pose
        "RGBD/CreateOccupancyGrid":        "false", # not needed for trajectory eval
        "RGBD/LinearUpdate":               "0",     # process every frame
        "RGBD/AngularUpdate":              "0",     # process every frame
        "RGBD/OptimizeMaxError":           "1",     # Table 2 default
    }

    # camera_info lives in the stereo_preproc namespace so that
    # image_transport::CameraSubscriber inside rectify_node can find it
    # (it auto-deduces the camera_info topic from the image topic namespace).
    remappings = [
        ("left/image_rect",  "/cam0/image_rect"),
        ("left/camera_info", "/cam0/camera_info"),
        ("right/image_rect", "/cam1/image_rect"),
        ("right/camera_info","/cam1/camera_info"),
    ]

    # ── Optional experiment-config parameter override ─────────────────────────
    # eval.py writes a ROS2 node-parameter YAML to this path and sets the env
    # var before launching so that experiment_config.yaml drives every run.
    # When the env var is unset the launch file uses common_params as-is.
    _exp_params_file = os.environ.get("RTABMAP_EXP_PARAMS", "")
    def _node_params(extra=None):
        params = [common_params]
        if _exp_params_file and os.path.exists(_exp_params_file):
            params.append(_exp_params_file)
        if extra:
            params.append(extra)
        return params

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),

            Node(
                package="rtabmap_odom",
                executable="stereo_odometry",
                output="screen",
                # always_process_most_recent_frame=False: process every frame in
                # arrival order (correct for offline benchmark — matches paper setup).
                # Frame drops are handled by reducing Kp/MaxFeatures + OdomF2M/MaxSize
                # in experiment_config.yaml so the algorithm keeps up with the bag rate.
                parameters=_node_params(
                    {
                        "always_process_most_recent_frame": False,
                        "publish_null_when_lost": False,
                    }
                ),
                remappings=remappings,
                arguments=["-d"],
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=_node_params(),
                remappings=remappings,
                arguments=["-d"],
            ),

            ExecuteProcess(
                cmd=[
                    "python3",
                    "/workspace/euroc_stereo_preproc.py",
                    "--ros-args",
                    "-p",
                    "use_sim_time:=True",
                ],
                output="screen",
            ),

            # EuRoC example TF tree.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[
                    "--x", "0",
                    "--y", "0",
                    "--z", "0",
                    "--roll", "3.1415926",
                    "--pitch", "-1.570796",
                    "--yaw", "0",
                    "--frame-id", "base_link",
                    "--child-frame-id", "imu4",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[
                    "--x", "-0.021640",
                    "--y", "-0.064677",
                    "--z", "0.009811",
                    "--roll", "1.555925",
                    "--pitch", "0.025777",
                    "--yaw", "0.003757",
                    "--frame-id", "imu4",
                    "--child-frame-id", "cam0",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[
                    "--x", "-0.019844",
                    "--y", "0.045369",
                    "--z", "0.007862",
                    "--roll", "1.558237",
                    "--pitch", "0.025393",
                    "--yaw", "0.017907",
                    "--frame-id", "imu4",
                    "--child-frame-id", "cam1",
                ],
            ),
        ]
    )
