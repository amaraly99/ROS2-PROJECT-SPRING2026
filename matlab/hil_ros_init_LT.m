% hil_ros_init_LT.m  (Lightweight)
% STEP 1 (LT): Run this immediately after opening MATLAB.
% Do NOT open Simulink before this. Do NOT run clear all after this.
%
% LT vs full version:
%   - sub_debug (/visp/debug_image) REMOVED — Pi no longer publishes it by default
%   - sub_vo    (/vo_pose) ADDED    — logs ov2slam output for paper experiments
%   - State timer at 20 Hz (same as full — Pi backward traffic is now eliminated
%     so there is no cost reason to run slower)
%
% LT startup sequence:
%   1. hil_ros_init_LT
%   2. Open + run hil_closed_loop.slx  (pacing = 1 s wall clock = real time)
%   3. sim_camera_publisher_timer_LT   (set PUBLISH_HZ in that script first)
%   4. live_camera_view_LT             (optional — single panel, 5 Hz)

disp('=== HIL ROS init (LT) starting ===')

setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');
setenv('ROS_DOMAIN_ID', '0');

if exist('node','var') && isvalid(node)
    disp('Node already alive, reusing.')
else
    node = ros2node('/matlab_bridge');
    pause(3);
end

topics = ros2('topic','list');
assert(any(contains(topics,'/rosout')), 'ROS failed. Restart MATLAB and rerun.');
disp('ROS OK')

% Camera publisher — BEST_EFFORT to match Pi subscriber QoS
% Intrinsics: fx=fy=1200 px, cx=320, cy=240, HFOV=29.9 deg.
% NOT 554/60deg — that was an assumption written into the configs and never
% checked against the block. Confirmed 2026-08-24 two independent ways:
% check_camera_fov (warp test, r=0.94) and check_stereo_geometry (fx=1236 from
% disparity vs ground-truth depth). See camera-focal-length memory.
cam_pub = ros2publisher(node, '/sim/camera/image_raw', 'sensor_msgs/Image', ...
    'Reliability','besteffort','Durability','volatile','Depth',5);

cam_msg = ros2message('sensor_msgs/Image');
cam_msg.height       = uint32(480);
cam_msg.width        = uint32(640);
cam_msg.encoding     = 'bgr8';
cam_msg.is_bigendian = uint8(0);
cam_msg.step         = uint32(640 * 3);

% ── Stereo right eye (optional) ──────────────────────────────────────────────
% The LEFT topic above is deliberately NOT renamed. /sim/camera/image_raw is
% hardcoded in visp_servo_node.cpp:398 and referenced by run_stack_hil.sh,
% record_bag.sh, bag_qos_overrides.yaml and hil_simulation.launch.py — and every
% existing rosbag replays against that name. The right eye is purely additive.
%
% STEREO_ON is set by set_stereo.m, which ALSO comments/uncomments the right
% camera blocks in the Simulink model — so the model and this script cannot
% disagree about which mode you are in. Default 0 means this whole block is
% skipped and the session is exactly the mono session that produced every
% published result.
if ~exist('STEREO_ON','var') || isempty(STEREO_ON), STEREO_ON = 0; end
assignin('base','STEREO_ON', double(STEREO_ON));

if STEREO_ON
    cam_pub_right = ros2publisher(node, '/sim/camera/right/image_raw', 'sensor_msgs/Image', ...
        'Reliability','besteffort','Durability','volatile','Depth',5);

    cam_msg_right              = ros2message('sensor_msgs/Image');
    cam_msg_right.height       = uint32(480);
    cam_msg_right.width        = uint32(640);
    cam_msg_right.encoding     = 'bgr8';   % NOT rgb8. Must match the left eye exactly:
                                           % the publisher packs B,G,R and the Pi calls
                                           % toCvCopy(msg,"rgb8"), which CONVERTS from
                                           % whatever the message declares. Declaring
                                           % rgb8 here while packing BGR would swap red
                                           % and blue in the right eye only — stereo
                                           % matching would degrade with no error.
    cam_msg_right.is_bigendian = uint8(0);
    cam_msg_right.step         = uint32(640 * 3);

    disp('Stereo ON  — right eye on /sim/camera/right/image_raw')
else
    cam_pub_right = [];
    cam_msg_right = [];
    disp('Stereo OFF — mono only on /sim/camera/image_raw')
end

% cmd_vel subscriber — writes Pi commands to base workspace
assignin('base','sim_vx', 0);
assignin('base','sim_vy', 0);
assignin('base','sim_vz', 0);
assignin('base','sim_wy', 0);
assignin('base','sim_wz', 0);
% Batch vector + version counter read by read_cmdvel_live_interp.m.
assignin('base','sim_cmdvel',     zeros(5,1));
assignin('base','sim_cmdvel_ver', int32(0));
assignin('base','last_cmdvel_print_time', 0);
assignin('base','last_cmdvel_values', [999 999 999 999 999]);

sub_cmdvel = ros2subscriber(node, '/cmd_vel', 'geometry_msgs/Twist', ...
    @cmdvel_callback, ...
    'Reliability','reliable','Durability','volatile');

% State variables read by the 20 Hz publish_state_LT timer
assignin('base','sim_pitch_angle', 0);
assignin('base','sim_pose',        zeros(5,1));
assignin('base','sim_heartbeat',   0);

pitch_pub = ros2publisher(node, '/sim/pitch_angle', 'std_msgs/Float64', ...
    'Reliability','reliable','Durability','volatile');
pitch_msg = ros2message('std_msgs/Float64');

pose_pub = ros2publisher(node, '/sim/drone_pose', 'std_msgs/Float64MultiArray', ...
    'Reliability','reliable','Durability','volatile');
pose_msg = ros2message('std_msgs/Float64MultiArray');

heartbeat_pub = ros2publisher(node, '/sim/heartbeat', 'std_msgs/Float64', ...
    'Reliability','reliable','Durability','volatile');
heartbeat_msg = ros2message('std_msgs/Float64');

target_pose_pub = ros2publisher(node, '/sim/target_pose', 'std_msgs/Float64MultiArray', ...
    'Reliability','reliable','Durability','transientlocal');
target_pose_msg = ros2message('std_msgs/Float64MultiArray');
target_pose_msg.data = double([35.5; 23.7; 3.2; pi]);
send(target_pose_pub, target_pose_msg);

% /vo_pose subscriber — receives ov2slam output for SLAM experiment logging.
% Stores [x; y; z; qx; qy; qz; qw] in sim_vo_pose for comparison with
% Simulink ground truth (sim_pose). Used in the SLAM-config→controller-quality
% and latency decomposition experiments.
% NOTE: topic will be absent (no messages) if ./run_stack_hil.sh is run without
% SLAM (--no-slam). Subscriber is safe to leave — it just never fires.
assignin('base','sim_vo_pose', zeros(7,1));
assignin('base','sim_vo_pose_count', 0);

sub_vo = ros2subscriber(node, '/vo_pose', 'geometry_msgs/PoseStamped', ...
    @vo_pose_callback, ...
    'Reliability','reliable','Durability','volatile');

old_ptimer = timerfindall('Name','pitch_pub_timer_LT');
if ~isempty(old_ptimer), stop(old_ptimer); delete(old_ptimer); end

% State timer at 20 Hz — visp_servo and ov2slam both benefit from full rate
pitch_pub_timer_LT = timer( ...
    'Name',          'pitch_pub_timer_LT', ...
    'Period',        0.05, ...
    'ExecutionMode', 'fixedRate', ...
    'TimerFcn',      @(~,~) publish_state_LT(pitch_pub, pitch_msg, ...
                                              pose_pub, pose_msg, ...
                                              heartbeat_pub, heartbeat_msg, ...
                                              target_pose_pub, target_pose_msg));
start(pitch_pub_timer_LT);

disp('State publisher (LT) started at 20 Hz on /sim/pitch_angle, /sim/drone_pose, /sim/heartbeat')
disp('/vo_pose subscriber active — sim_vo_pose updated when SLAM is running')
disp('=== LT Init complete. Open Simulink, run sim, then run sim_camera_publisher_timer_LT.m ===')

% ─────────────────────────────────────────────────────────────────────────────

function cmdvel_callback(msg)
    vx = double(msg.linear.x);
    vy = double(msg.linear.y);
    vz = double(msg.linear.z);
    wy = double(msg.angular.y);
    wz = double(msg.angular.z);
    assignin('base','sim_vx', vx);
    assignin('base','sim_vy', vy);
    assignin('base','sim_vz', vz);
    assignin('base','sim_wy', wy);
    assignin('base','sim_wz', wz);
    % Batch write + version bump for read_cmdvel_live_interp cache.
    assignin('base','sim_cmdvel', [vx; vy; vz; wy; wz]);
    ver = evalin('base','sim_cmdvel_ver');
    assignin('base','sim_cmdvel_ver', int32(ver) + int32(1));
    try
        now_t     = posixtime(datetime('now'));
        last_t    = evalin('base','last_cmdvel_print_time');
        last_vals = evalin('base','last_cmdvel_values');
        current_vals = [vx vy vz wy wz];
        if (now_t - last_t > 2.0) || any(abs(current_vals - last_vals) > 0.05)
            fprintf('[cmdvel] vx=%.2f vy=%.2f vz=%.2f wy=%.2f wz=%.2f\n', ...
                    vx, vy, vz, wy, wz);
            assignin('base','last_cmdvel_print_time', now_t);
            assignin('base','last_cmdvel_values', current_vals);
        end
    catch
    end
end

function vo_pose_callback(msg)
    persistent vo_count
    if isempty(vo_count), vo_count = 0; end
    try
        p = msg.pose.position;
        q = msg.pose.orientation;
        assignin('base','sim_vo_pose', [p.x; p.y; p.z; q.x; q.y; q.z; q.w]);
        vo_count = vo_count + 1;
        assignin('base','sim_vo_pose_count', vo_count);
    catch
    end
end

function publish_state_LT(pitch_pub, pitch_msg, pose_pub, pose_msg, ...
                           heartbeat_pub, heartbeat_msg, ...
                           target_pose_pub, target_pose_msg)
    try
        pitch_msg.data = double(evalin('base', 'sim_pitch_angle'));
        pose_msg.data  = double(evalin('base', 'sim_pose'));
        send(pitch_pub, pitch_msg);
        send(pose_pub,  pose_msg);
    catch
    end
    try
        heartbeat_msg.data = double(evalin('base', 'sim_heartbeat'));
        send(heartbeat_pub, heartbeat_msg);
    catch
    end

    % Re-publish target_pose at ~1 Hz (every 20 ticks at 20 Hz)
    persistent target_tick
    if isempty(target_tick), target_tick = 0; end
    target_tick = target_tick + 1;
    if mod(target_tick, 20) == 0
        try
            send(target_pose_pub, target_pose_msg);
        catch
        end
    end
end
