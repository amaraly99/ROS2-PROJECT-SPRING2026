# hil_simulation.launch.py — HIL perception + controller stack
#
# All configuration is driven by env vars in run_stack_hil.sh (which sets the
# launch args below). Can also be invoked directly:
#
#   ros2 launch sim_camera_bridge hil_simulation.launch.py \
#       controller:=ibvs detector:=oracle slam:=false
#
# Args:
#   controller      ibvs | proportional | h_vs   (default: proportional)
#   detector        yolo | oracle                 (default: yolo)
#                     yolo   → starts yolo_bridge (reads /yolo_shm from host yolo_producer)
#                     oracle → starts oracle_detector_node (projects target from drone pose)
#   slam            true | false  (default: false — OV2SLAM started separately by run_stack_hil.sh)
#   ovcam           true | false  (default: true — ovcam_bridge for SLAM/debug)
#   debug_image     true | false  (default: false)
#   benchmark_mode  true | false  (default: false = scout, engage immediately on boot)
#   target_class    COCO label string  (default: stop sign)
#   calib_yaml      OV2SLAM calibration file path
#   controller_cpu  taskset core(s) for the controller node  (e.g. "0"; empty = no pin)
#   detector_cpu    taskset core(s) for yolo_bridge/oracle node  (empty = no pin)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node


CTRL_MAP = {
    # controller_name: (package, executable, gains_yaml)
    'proportional': ('hil_servo',  'hil_servo_node',   'bench_proportional.yaml'),
    'ibvs':         ('visp_servo', 'visp_servo_node',  'bench_ibvs.yaml'),
    'h_vs':         ('h_vs_servo', 'h_vs_servo_node',  'bench_h_vs.yaml'),
    'pbvs':         ('visp_pbvs_servo', 'visp_pbvs_node', 'bench_pbvs.yaml')
}

DET_MAP = {
    # detector_name: (package, executable, node_name, [param_yaml, ...], wants_target_class)
    # Every detector MUST publish /yolo/detections (yolo_msgs/DetectionArray). To add one,
    # add a row here. Hardware-backed detectors that need a HOST producer (like yolo's
    # yolo_producer on the Hailo NPU) wire that host process in run_stack_hil.sh, not here.
    'yolo':   ('yolo_bridge',     'yolo_bridge_node',     'yolo_bridge',     [],                    False),
    'oracle': ('oracle_detector', 'oracle_detector_node', 'oracle_detector', ['bench_oracle.yaml'], True),
}


def launch_setup(context, *args, **kwargs):
    def s(name): return LaunchConfiguration(name).perform(context)

    workspace      = s('workspace')
    target_class   = s('target_class')
    controller     = s('controller')
    detector       = s('detector')
    slam           = s('slam').lower() == 'true'
    ovcam          = s('ovcam').lower() == 'true'
    debug_image    = s('debug_image').lower() == 'true'
    benchmark_mode = s('benchmark_mode').lower() in ('true', '1', 'yes')
    calib_yaml     = s('calib_yaml')
    controller_cpu = s('controller_cpu').strip()
    detector_cpu   = s('detector_cpu').strip()
    use_slam_depth = s('use_slam_depth').lower() in ('true', '1', 'yes')

    cfg = lambda name: f'{workspace}/config/hil/{name}'
    # taskset prefix for a node, or None when the cpu arg is empty (no pinning).
    # MUST be a space-separated STRING — launch_ros concatenates a list with NO
    # spaces (['taskset','-c','0'] → 'taskset-c0', a bogus command). Matches the
    # inline ov2slam node's prefix='taskset -c 2,3'.
    pfx = lambda cpu: f'taskset -c {cpu}' if cpu else None

    nodes = []

    # ── 1. SHM filler — converts /sim/camera/image_raw → /dev/shm/ovcam_frames ──
    nodes.append(Node(
        package='sim_camera_bridge',
        executable='sim_camera_bridge_node',
        name='sim_camera_bridge',
        output='screen',
        parameters=[{
            'input_topic': '/sim/camera/image_raw',
            'width': 640, 'height': 480, 'slots': 4,
            'shm_name': '/ovcam_frames',
            'sem_name': '/ovcam_ready',
        }],
    ))

    # ── 2. ovcam_bridge — SHM → /ovcam/image_raw (needed by OV2SLAM / debug) ───
    if ovcam:
        nodes.append(Node(
            package='ovcam_bridge',
            executable='ovcam_bridge_node',
            name='ovcam_bridge',
            output='screen',
        ))

    # ── 3. Detector (DET_MAP-driven; exactly one runs) ───────────────────────────
    #   oracle → projects the known target through the sim drone pose (needs
    #            /sim/drone_pose + /sim/target_pose from MATLAB; no hardware).
    #   yolo   → yolo_bridge reads /yolo_shm written by the host yolo_producer.
    # Both publish /yolo/detections. Add a detector by adding a DET_MAP row above.
    if detector not in DET_MAP:
        raise RuntimeError(
            f"Unknown detector='{detector}'. Valid: {list(DET_MAP)}")
    dpkg, dexe, dname, dparam_files, dwants_class = DET_MAP[detector]
    dparams = [cfg(p) for p in dparam_files]
    if dwants_class:
        dparams.append({'target_class': target_class})
    nodes.append(Node(
        package=dpkg,
        executable=dexe,
        name=dname,
        output='screen',
        prefix=pfx(detector_cpu),
        parameters=dparams,
    ))

    # ── 4. OV2SLAM — normally deferred by run_stack_hil.sh (slam:=false passed). ─
    #    Only active when this launch file is invoked directly with slam:=true.
    if slam:
        nodes.append(Node(
            package='ov2slam',
            executable='ov2slam_node',
            name='ov2slam',
            output='screen',
            prefix='taskset -c 2,3',
            arguments=[calib_yaml],
        ))

    # ── 5. Controller ─────────────────────────────────────────────────────────────
    if controller not in CTRL_MAP:
        raise RuntimeError(
            f"Unknown controller='{controller}'. Valid: {list(CTRL_MAP)}")
    pkg, exe, ctrl_cfg = CTRL_MAP[controller]
    nodes.append(Node(
        package=pkg,
        executable=exe,
        name=exe,
        output='screen',
        prefix=pfx(controller_cpu),
        parameters=[cfg('bench_fsm.yaml'), cfg(ctrl_cfg),
                    {'target_class': target_class,
                     'benchmark_mode': benchmark_mode,
                     'use_slam_depth': use_slam_depth}],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='proportional',
            description='Servoing law: ibvs (TS1) | proportional (TS2) | h_vs (TS3) | pbvs (TS4)'),
        DeclareLaunchArgument('detector', default_value='yolo',
            description='yolo — Hailo NPU via yolo_bridge; oracle — synthetic bbox from pose'),
        DeclareLaunchArgument('workspace',
            default_value=EnvironmentVariable('WORKSPACE_DIR', default_value='/workspace')),
        DeclareLaunchArgument('target_class', default_value='stop sign',
            description='COCO class label(s) to track (comma-separated)'),
        DeclareLaunchArgument('slam', default_value='false',
            description='Start OV2SLAM inline (normally run_stack_hil.sh starts it separately)'),
        DeclareLaunchArgument('ovcam', default_value='true',
            description='Start ovcam_bridge (needed when slam=true or debug_image=true)'),
        DeclareLaunchArgument('debug_image', default_value='false',
            description='Publish /visp/debug_image back to MATLAB (costly — 921 KB/frame)'),
        DeclareLaunchArgument('benchmark_mode', default_value='false',
            description='true = wait for sim reset before engaging (bench); false = scout immediately'),
        DeclareLaunchArgument('controller_cpu', default_value='',
            description='taskset core(s) for the controller node (e.g. "0"); empty = no pin'),
        DeclareLaunchArgument('detector_cpu', default_value='',
            description='taskset core(s) for yolo_bridge/oracle_detector node; empty = no pin'),
        DeclareLaunchArgument('use_slam_depth', default_value='false',
            description='IBVS only: feed SLAM map-point depth into the interaction matrix (vs bbox depth)'),
        DeclareLaunchArgument('calib_yaml',
            default_value=PathJoinSubstitution(
                [EnvironmentVariable('WORKSPACE_DIR', default_value='/workspace'),
                 'camera_calib', 'hil_sim_ov2slam.yaml']),
            description='OV2SLAM calibration YAML'),
        OpaqueFunction(function=launch_setup),
    ])
