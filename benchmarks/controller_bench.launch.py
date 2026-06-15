# ─────────────────────────────────────────────────────────────────
# controller_bench.launch.py — YOLO-agnostic controller benchmark.
#
# Launches ONLY:
#   - oracle_detector            (perfect bbox → /yolo/detections)
#   - ONE controller node        (TS1 visp_servo OR TS2 hil_servo)
#
# No camera bridge, no YOLO, no SLAM — the oracle replaces all perception,
# so the controller's only inputs are MATLAB's /sim/* topics + the oracle.
#
# Invoke (inside the container, CycloneDDS env already exported by the harness):
#   ros2 launch /workspace/benchmarks/controller_bench.launch.py \
#        controller:=ibvs   workspace:=/workspace
#   ros2 launch /workspace/benchmarks/controller_bench.launch.py \
#        controller:=proportional workspace:=/workspace
#
# Only ONE controller runs per invocation (both publish /cmd_vel).
# ─────────────────────────────────────────────────────────────────
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                   PythonExpression, EnvironmentVariable)
from launch_ros.actions import Node


def generate_launch_description():
    controller   = LaunchConfiguration('controller')
    workspace    = LaunchConfiguration('workspace')
    target_class = LaunchConfiguration('target_class')
    ctrl_core    = LaunchConfiguration('ctrl_core')

    cfg = lambda name: PathJoinSubstitution([workspace, 'config', 'hil', name])

    is_ibvs = IfCondition(PythonExpression(["'", controller, "' == 'ibvs'"]))
    is_prop = IfCondition(PythonExpression(["'", controller, "' == 'proportional'"]))

    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='proportional',
            description="Test subject: 'ibvs' (TS1) or 'proportional' (TS2)"),
        DeclareLaunchArgument('workspace',
            default_value=EnvironmentVariable('WORKSPACE_DIR', default_value='/workspace')),
        DeclareLaunchArgument('target_class', default_value='stop sign'),
        DeclareLaunchArgument('ctrl_core', default_value='0',
            description='CPU core to pin the controller to (clean pidstat reading)'),

        # Oracle — perfect bbox from ground-truth pose.
        Node(
            package='oracle_detector',
            executable='oracle_detector_node',
            name='oracle_detector',
            output='screen',
            parameters=[cfg('bench_oracle.yaml'),
                        {'target_class': target_class}],
        ),

        # TS1 — IBVS (ViSP vpServo). Shared FSM + IBVS law.
        Node(
            package='visp_servo',
            executable='visp_servo_node',
            name='visp_servo_node',
            output='screen',
            condition=is_ibvs,
            prefix=['taskset -c ', ctrl_core],
            parameters=[cfg('bench_fsm.yaml'), cfg('bench_ibvs.yaml'),
                        {'target_class': target_class}],
        ),

        # TS2 — proportional. Shared FSM + proportional law.
        Node(
            package='hil_servo',
            executable='hil_servo_node',
            name='hil_servo_node',
            output='screen',
            condition=is_prop,
            prefix=['taskset -c ', ctrl_core],
            parameters=[cfg('bench_fsm.yaml'), cfg('bench_proportional.yaml'),
                        {'target_class': target_class}],
        ),
    ])