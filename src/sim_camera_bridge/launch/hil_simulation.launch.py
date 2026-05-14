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
    target_class = LaunchConfiguration('target_class')
    workspace    = LaunchConfiguration('workspace')
    slam         = LaunchConfiguration('slam')

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_class', default_value='stop sign',
            description='COCO class(es) for ViSP to track — comma-separated list'),
        DeclareLaunchArgument(
            'workspace', default_value=EnvironmentVariable('WORKSPACE_DIR',
                default_value='/workspace'),
            description='Repo root inside the container (default: /workspace)'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='Set to false to skip OV2SLAM (useful for DDS-only testing)'),

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
            }],
        ),

        # 2. ovcam_bridge — unchanged from main; reads SHM, publishes /ovcam/image_raw mono8
        Node(
            package='ovcam_bridge',
            executable='ovcam_bridge_node',
            name='ovcam_bridge',
            output='screen',
        ),

        # 3. yolo_bridge — unchanged from main; reads /yolo_shm, publishes /yolo/detections
        Node(
            package='yolo_bridge',
            executable='yolo_bridge_node',
            name='yolo_bridge',
            output='screen',
        ),

        # 4. OV2SLAM — same yaml as main pipeline (intrinsics + topic match HIL)
        Node(
            package='ov2slam',
            executable='ov2slam_node',
            name='ov2slam',
            output='screen',
            condition=IfCondition(slam),
            arguments=[PathJoinSubstitution(
                [workspace, 'camera_calib', 'ov5647_ov2slam.yaml'])],
        ),

        # 5. visp_servo — HIL params; target_class defaults to "stop sign"
        Node(
            package='visp_servo',
            executable='visp_servo_node',
            name='visp_servo_node',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [workspace, 'config', 'hil', 'hil_servo_params.yaml']),
                {'target_class': target_class},
            ],
        ),
    ])
