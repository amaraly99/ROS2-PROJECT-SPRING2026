# hil_simulation.launch.py — HIL mode for sim-hil branch
#
# Starts the ROS2 portion of the perception stack against a synthetic
# camera stream from MATLAB/Simulink/Unreal NYC scene. The native host
# binary `yolo_producer` is started separately by run_stack_hil.sh — it
# attaches to the /ovcam_frames POSIX SHM that sim_camera_bridge fills.
#
# What MATLAB must publish (cross-host on the same LAN, ROS_DOMAIN_ID=0):
#   topic     /sim/camera/image_raw
#   type      sensor_msgs/Image
#   encoding  rgb8
#   size      640x480
#   rate      ~20 Hz
#   QoS       RELIABILE + VOLATILE + KEEP_LAST(10)
#   DDS       matches RMW_IMPLEMENTATION on RPi5 (cyclonedds default,
#             fastrtps via run_stack_hil.sh DDS=fastrtps option)
#
# Argument:
#   target_class:=<coco_label>   default: stop sign (parking-lot scene)
#
# This launch file does not start sensors — sim_camera_bridge writes
# directly into the SHM that ovcam_producer would fill on main.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    target_class  = LaunchConfiguration('target_class')
    workspace     = LaunchConfiguration('workspace')
    slam          = LaunchConfiguration('slam')
    ovcam         = LaunchConfiguration('ovcam')
    debug_image   = LaunchConfiguration('debug_image')
    calib_yaml    = LaunchConfiguration('calib_yaml')
    slam_cores    = LaunchConfiguration('slam_cores')
    class_name_map = LaunchConfiguration('class_name_map')
    cam_stamp_log  = LaunchConfiguration('cam_stamp_log')

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_class', default_value='stop sign',
            description='COCO class(es) for ViSP to track — comma-separated list'),
        DeclareLaunchArgument(
            'class_name_map', default_value='',
            description='yolo_bridge custom class-id→name map, "id:name,id:name" '
                        '(non-YOLO detectors, e.g. "0:object" or "100:vlm_detection")'),
        DeclareLaunchArgument(
            'cam_stamp_log', default_value='',
            description='CSV path for sim_camera_bridge per-frame stamp log '
                        '(MATLAB stamp, recv, SHM write — latency stages S1/S2). Empty = off'),
        DeclareLaunchArgument(
            'workspace', default_value=EnvironmentVariable('WORKSPACE_DIR',
                default_value='/workspace'),
            description='Repo root inside the container (default: /workspace)'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='Set to false to skip OV2SLAM (useful for DDS-only testing)'),
        DeclareLaunchArgument(
            'slam_cores', default_value='2,3',
            description='taskset CPU list for OV2SLAM core pinning (sweepable per run)'),
        DeclareLaunchArgument(
            'ovcam', default_value='true',
            description='Set to false to skip ovcam_bridge (safe only when slam:=false and debug_image:=false)'),
        DeclareLaunchArgument(
            'debug_image', default_value='false',
            description='Publish /visp/debug_image back to MATLAB (expensive — 921 KB/frame)'),
        DeclareLaunchArgument(
            'calib_yaml',
            default_value=PathJoinSubstitution(
                [EnvironmentVariable('WORKSPACE_DIR', default_value='/workspace'),
                 'camera_calib', 'hil_sim_ov2slam.yaml']),
            description='OV2SLAM calibration YAML path — override to swap configs without editing this file'),

        # 1. SHM filler — replaces ovcam_producer in HIL
        Node(
            package='sim_camera_bridge',
            executable='sim_camera_bridge_node',
            name='sim_camera_bridge',
            output='screen',
            parameters=[{
                'input_topic': '/sim/camera/image_raw',
                'width':  640,
                'height': 480,
                'slots':  4,
                'shm_name': '/ovcam_frames',
                'sem_name': '/ovcam_ready',
                'stamp_log': cam_stamp_log,
            }],
        ),

        # 2. ovcam_bridge — reads SHM, publishes /ovcam/image_raw mono8
        #    Only needed when slam:=true or debug_image:=true; skip otherwise.
        Node(
            package='ovcam_bridge',
            executable='ovcam_bridge_node',
            name='ovcam_bridge',
            output='screen',
            condition=IfCondition(ovcam),
        ),

        # 3. yolo_bridge — unchanged from main; reads /yolo_shm, publishes /yolo/detections
        Node(
            package='yolo_bridge',
            executable='yolo_bridge_node',
            name='yolo_bridge',
            output='screen',
            parameters=[{'class_name_map': class_name_map}],
        ),

        # 4. OV2SLAM — HIL-specific calibration via calib_yaml arg (default: hil_sim_ov2slam.yaml).
        #    Pinned to cores 2,3. Normally started by run_stack_hil.sh via ros2 run (slam:=false
        #    is passed to this launch file) so this node block is only used for direct ros2 launch
        #    invocations. The calib_yaml arg allows swapping configs without editing this file.
        Node(
            package='ov2slam',
            executable='ov2slam_node',
            name='ov2slam',
            output='screen',
            # Core pinning via a substitution-list prefix so the mask is taken from
            # the `slam_cores` launch arg (was hardcoded 'taskset -c 2,3', which the
            # single-branch sweep could not override). taskset pins this exec and its
            # children to the given CPU list.
            prefix=['taskset -c ', slam_cores],
            condition=IfCondition(slam),
            arguments=[calib_yaml],
        ),

        # 5. visp_servo — HIL params; target_class defaults to "stop sign"
        #    debug_image_enabled overrides the YAML default so the launch arg
        #    takes precedence without requiring a YAML edit.
        Node(
            package='visp_servo',
            executable='visp_servo_node',
            name='visp_servo_node',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [workspace, 'config', 'hil', 'hil_servo_params.yaml']),
                {'target_class': target_class,
                 'debug_image_enabled': debug_image},
            ],
        ),
    ])
