% live_debug_view_LT.m  (Lightweight, two-panel)
% Shows the Pi's own detector + SLAM debug images side by side, LIVE, over
% the network. Unlike live_camera_view_LT / _stereo_LT (which read local
% Simulink workspace variables — zero network traffic), this one is a real
% ROS2 subscriber: it pulls two image topics FROM the Pi.
%
%   LEFT  panel: /yolo/annotated   — YOLO's own boxes drawn on the frame
%                                     (published by yolo_bridge_node, always on)
%   RIGHT panel: image_track       — OV2SLAM's own tracked-keypoints overlay
%                                     (published by ov2slam_node — it only does
%                                     the drawing work once something is
%                                     subscribed, so opening this figure is
%                                     what turns that overlay on in the first
%                                     place; free otherwise)
%
% Requires hil_ros_init_LT to have run first (needs the `node` it creates).
% Does NOT require the stereo right eye, SLAM depth, or any particular
% controller — both topics exist on every config that runs yolo_bridge +
% ov2slam.
%
% Both topics publish rgb8 (confirmed from source: yolo_bridge_node.cpp:194,
% ros_visualizer.hpp:132) — rosReadImage handles the conversion, no manual
% channel-swap needed here (contrast with sim_camera_publisher_timer_LT.m,
% which DOES need a manual RGB->BGR swap for the outbound sim frame — that's
% a different direction, not relevant to this inbound path).
%
% QoS must match each publisher or the subscriber connects but silently never
% receives anything (a DDS reliable subscriber cannot match a best-effort
% publisher):
%   /yolo/annotated -> BEST_EFFORT, depth 1   (yolo_bridge_node.cpp:57-58)
%   image_track     -> RELIABLE,    depth 1000 (ros_visualizer.hpp:77, default QoS)
%
% To STOP:    stop(live_debug_view_timer_LT)   — or just close the figure.

if ~exist('node','var')
    error('node missing. Run hil_ros_init_LT first.');
end

old = timerfindall('Name','live_debug_view_timer_LT');
if ~isempty(old), stop(old); delete(old); end

% Latest-message cache, written by the ROS callbacks (cheap: no image decode
% there), read by the timer (decode + draw at a controlled rate). Same
% split as every other _LT viewer in this repo — keeps GUI work off the ROS
% callback thread.
assignin('base', 'latest_yolo_annotated_msg', []);
assignin('base', 'latest_image_track_msg',    []);

sub_yolo_annotated_LT = ros2subscriber(node, "/yolo/annotated", "sensor_msgs/Image", ...
    @(msg) assignin('base', 'latest_yolo_annotated_msg', msg), ...
    "Reliability", "besteffort", "Durability", "volatile", "Depth", 1);

sub_image_track_LT = ros2subscriber(node, "image_track", "sensor_msgs/Image", ...
    @(msg) assignin('base', 'latest_image_track_msg', msg), ...
    "Reliability", "reliable", "Durability", "volatile", "Depth", 1000);

% Keep the subscriber objects alive in the base workspace — MATLAB stops
% listening the instant they're garbage-collected, same reason cam_pub /
% cam_msg live in base workspace rather than this script's own scope.
assignin('base', 'sub_yolo_annotated_LT', sub_yolo_annotated_LT);
assignin('base', 'sub_image_track_LT',    sub_image_track_LT);

fig = figure('Name','Debug: YOLO detections | SLAM keypoints (LT)','NumberTitle','off',...
             'MenuBar','none','ToolBar','none',...
             'Position',[80 120 1360 540]);

ax_l = subplot(1,2,1,'Parent',fig);
h_l  = imshow(zeros(480,640,3,'uint8'),'Parent',ax_l);
title(ax_l,'YOLO detections  ->  /yolo/annotated');

ax_r = subplot(1,2,2,'Parent',fig);
h_r  = imshow(zeros(480,640,3,'uint8'),'Parent',ax_r);
title(ax_r,'SLAM keypoints  ->  image\_track');

live_debug_view_timer_LT = timer(...
    'Name',          'live_debug_view_timer_LT',...
    'Period',        0.2,...
    'ExecutionMode', 'fixedRate',...
    'TimerFcn',      @(~,~) update_debug_view_LT(h_l, h_r, fig));

start(live_debug_view_timer_LT);
disp('Live debug view (LT) started — 5 Hz, two panels, live from the Pi over ROS2.')
disp('  LEFT:  /yolo/annotated  (always publishing)')
disp('  RIGHT: image_track      (SLAM only draws once this is watching — give it a few seconds)')
disp('Close figure or stop(live_debug_view_timer_LT) to stop.')

function update_debug_view_LT(h_l, h_r, fig)
    if ~ishandle(fig)
        t = timerfindall('Name','live_debug_view_timer_LT');
        if ~isempty(t), stop(t); delete(t); end
        return;
    end
    try
        msg_l = evalin('base','latest_yolo_annotated_msg');
        if ~isempty(msg_l)
            set(h_l,'CData', rosReadImage(msg_l));
        end
    catch
    end
    try
        msg_r = evalin('base','latest_image_track_msg');
        if ~isempty(msg_r)
            set(h_r,'CData', rosReadImage(msg_r));
        end
    catch
    end
    drawnow limitrate;   % skips flush if GPU is already busy — no blocking
end
