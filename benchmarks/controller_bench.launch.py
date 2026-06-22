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
# CPU pinning (ctrl_core arg):
#   ctrl_core:=0        pin to core 0
#   ctrl_core:=0,1,2    pin to cores 0-2
#   ctrl_core:=none     no pinning (default)
#   ctrl_core:=         no pinning
#
# Only ONE controller runs per invocation (both publish /cmd_vel).
# ─────────────────────────────────────────────────────────────────
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                   EnvironmentVariable)
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    controller   = LaunchConfiguration('controller').perform(context)
    workspace    = LaunchConfiguration('workspace').perform(context)
    target_class = LaunchConfiguration('target_class').perform(context)
    ctrl_core    = LaunchConfiguration('ctrl_core').perform(context).strip()
    bench_mode   = LaunchConfiguration('benchmark_mode').perform(context).strip().lower()

    cfg = lambda name: f'{workspace}/config/hil/{name}'

    # benchmark_mode: 'true' waits for a fresh sim reset (clean t=0); 'false'
    # scouts immediately on boot (no Stop→Run). Accept true/1/yes as true.
    benchmark_mode = bench_mode in ('true', '1', 'yes')

    # Build taskset prefix: '' or 'none' → no pinning; else taskset -c <value>
    if ctrl_core and ctrl_core.lower() != 'none':
        prefix = ['taskset -c ', ctrl_core]
    else:
        prefix = []

    nodes = [
        Node(
            package='oracle_detector',
            executable='oracle_detector_node',
            name='oracle_detector',
            output='screen',
            parameters=[cfg('bench_oracle.yaml'), {'target_class': target_class}],
        ),
    ]

    if controller == 'ibvs':
        nodes.append(Node(
            package='visp_servo',
            executable='visp_servo_node',
            name='visp_servo_node',
            output='screen',
            prefix=prefix,
            parameters=[cfg('bench_fsm.yaml'), cfg('bench_ibvs.yaml'),
                        {'target_class': target_class,
                         'benchmark_mode': benchmark_mode}],
        ))
    elif controller == 'proportional':
        nodes.append(Node(
            package='hil_servo',
            executable='hil_servo_node',
            name='hil_servo_node',
            output='screen',
            prefix=prefix,
            parameters=[cfg('bench_fsm.yaml'), cfg('bench_proportional.yaml'),
                        {'target_class': target_class,
                         'benchmark_mode': benchmark_mode}],
        ))
    elif controller == 'h_vs':
        nodes.append(Node(
            package='h_vs_servo',
            executable='h_vs_servo_node',
            name='h_vs_servo_node',
            output='screen',
            prefix=prefix,
            parameters=[cfg('bench_fsm.yaml'), cfg('bench_h_vs.yaml'),
                        {'target_class': target_class,
                         'benchmark_mode': benchmark_mode}],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='proportional',
            description="Test subject: 'ibvs' (TS1), 'proportional' (TS2), or 'h_vs' (TS3)"),
        DeclareLaunchArgument('workspace',
            default_value=EnvironmentVariable('WORKSPACE_DIR', default_value='/workspace')),
        DeclareLaunchArgument('target_class', default_value='stop sign'),
        DeclareLaunchArgument('ctrl_core', default_value='none',
            description="CPU core(s) to pin the controller to: '0', '0,1,2', or 'none'/'' for no pinning"),
        DeclareLaunchArgument('benchmark_mode', default_value='true',
            description="'true' = benchmarking (wait for sim reset, clean t=0); "
                        "'false' = scouting (engage immediately on boot, no Stop→Run)"),
        OpaqueFunction(function=launch_setup),
    ])