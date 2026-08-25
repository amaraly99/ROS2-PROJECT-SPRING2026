% live_camera_view_stereo_LT.m  (Lightweight, two-panel)
% Shows both Simulink drone camera eyes side by side.
% latest_frame / latest_frame_right are written locally by Simulink — no
% network traffic is involved, so this is safe to leave running.
%
% Requires set_stereo(1) and a running model. For mono use live_camera_view_LT.
%
% WHAT TO LOOK FOR
%   * Same scene in both panels, colours natural in BOTH.
%       Odd colours in ONE eye only  -> an rgb8/bgr8 mismatch somewhere.
%   * The right panel shifted slightly LEFT relative to the left panel.
%       Objects sit at SMALLER u in the right eye, because the right camera is
%       displaced to the right. That is positive disparity, which is the
%       convention OV2SLAM expects.
%       Shifted the WRONG way -> the two cameras are swapped; check that the
%       right camera's Relative translation Y is NEGATIVE (ISO 8855: +Y is left).
%   * PIXEL-IDENTICAL panels -> the two cameras share a Sensor identifier and
%       Unreal is feeding both blocks the same sensor. Zero disparity, stereo
%       silently does nothing. Fix: right camera -> Mounting -> Sensor identifier = 2.
%
%   The shift is only obvious up close: disparity = fx*b/z = 554*0.11/z = 60.9/z px,
%   so ~20 px at 3 m but under 2 px at 35 m. Judge it on something near.
%
% For the numeric version of all of the above, run check_stereo_geometry.

old = timerfindall('Name','live_cam_viewer_stereo_LT');
if ~isempty(old), stop(old); delete(old); end

if ~evalin('base','exist(''latest_frame_right'',''var'')')
    error(['latest_frame_right missing. Run set_stereo(1) and restart the model ' ...
           '(use live_camera_view_LT for mono).']);
end

fig = figure('Name','Simulink Stereo Camera (LT)','NumberTitle','off',...
             'MenuBar','none','ToolBar','none',...
             'Position',[80 120 1360 540]);

ax_l = subplot(1,2,1,'Parent',fig);
h_l  = imshow(zeros(480,640,3,'uint8'),'Parent',ax_l);
title(ax_l,'LEFT  (latest\_frame)  ->  /sim/camera/image\_raw');

ax_r = subplot(1,2,2,'Parent',fig);
h_r  = imshow(zeros(480,640,3,'uint8'),'Parent',ax_r);
title(ax_r,'RIGHT (latest\_frame\_right)  ->  /sim/camera/right/image\_raw');

live_view_timer_stereo_LT = timer(...
    'Name',          'live_cam_viewer_stereo_LT',...
    'Period',        0.2,...
    'ExecutionMode', 'fixedRate',...
    'TimerFcn',      @(~,~) update_view_stereo_LT(h_l, h_r, fig));

start(live_view_timer_stereo_LT);
disp('Live stereo view (LT) started — 5 Hz, two panels. Close figure or stop(live_view_timer_stereo_LT) to stop.')

function update_view_stereo_LT(h_l, h_r, fig)
    if ~ishandle(fig)
        t = timerfindall('Name','live_cam_viewer_stereo_LT');
        if ~isempty(t), stop(t); delete(t); end
        return;
    end
    try
        fl = evalin('base','latest_frame');
        if ~isempty(fl), set(h_l,'CData',fl); end
    catch
    end
    try
        fr = evalin('base','latest_frame_right');
        if ~isempty(fr), set(h_r,'CData',fr); end
    catch
    end
    drawnow limitrate;   % skips flush if GPU is already busy — no blocking
end
