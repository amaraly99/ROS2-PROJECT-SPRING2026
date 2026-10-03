% hil_ros_init_LT_rgbd.m  (RGB-D add-on to hil_ros_init_LT.m, created 2026-09-27)
%
% Runs the unmodified hil_ros_init_LT (mono: colour publisher, cmd_vel, state
% timer, ...) and then adds ONE thing: the left-camera depth publisher.
% hil_ros_init_LT.m itself is not edited, so the mono/stereo path cannot change.
%
% RGB-D startup sequence:
%   1. set_rgbd(1)
%   2. hil_ros_init_LT_rgbd
%   3. start hil_closed_loop_baseline_RGBD
%   4. sim_camera_publisher_timer_LT_rgbd
%
% Depth contract (Pi: src/rtabmap_docker/hil_rgbd.launch.py):
%   /sim/camera/depth/image_raw, sensor_msgs/Image, 480x640, 16UC1 millimetres,
%   little-endian, row-major, invalid/sky/>65.535 m -> 0,
%   header.stamp IDENTICAL to the colour frame of the same tick,
%   frame_id 'camera' (= ovcam_bridge's frame_id on /ovcam/image_raw).

if ~exist('RGBD_ON','var') || isempty(RGBD_ON) || ~RGBD_ON
    error('RGBD_ON is not set. Run set_rgbd(1) first (after any clear all).');
end
STEREO_ON = 0;   % RGB-D uses the left camera only; never create a right-eye publisher

hil_ros_init_LT

% Color frame_id: hil_ros_init_LT leaves it empty because ovcam_bridge used to stamp
% 'camera' on the gray copy. In color mode (Pi stack config ..._color_...) RTAB-Map
% reads this topic directly, so it must carry the same frame as the depth.
cam_msg.header.frame_id = 'camera';

depth_pub = ros2publisher(node, '/sim/camera/depth/image_raw', 'sensor_msgs/Image', ...
    'Reliability','besteffort','Durability','volatile','Depth',5);

depth_msg = ros2message('sensor_msgs/Image');
depth_msg.height          = uint32(480);
depth_msg.width           = uint32(640);
depth_msg.encoding        = '16UC1';
depth_msg.is_bigendian    = uint8(0);        % typecast on x86 Windows is little-endian
depth_msg.step            = uint32(640 * 2);
depth_msg.header.frame_id = 'camera';        % ovcam_bridge_node.cpp stamps 'camera' on
                                             % /ovcam/image_raw, the colour RTAB-Map pairs with

disp('RGB-D ON: depth on /sim/camera/depth/image_raw (16UC1 mm)')
disp('=== RGB-D init complete. Start hil_closed_loop_baseline_RGBD, then run sim_camera_publisher_timer_LT_rgbd ===')
