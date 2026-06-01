% live_camera_view_LT.m  (Lightweight)
% Shows the raw Simulink drone camera only — no Pi debug panel.
% latest_frame is written locally by Simulink, no network traffic needed.
%
% LT changes vs full version:
%   - Single panel (no Pi debug image — no sub_debug callback or 921 KB/frame Pi→MATLAB traffic)
%   - 5 Hz refresh (drawnow limitrate skips if GPU is busy — no stutter)
%   - Timer auto-stops when figure is closed

old = timerfindall('Name','live_cam_viewer_LT');
if ~isempty(old), stop(old); delete(old); end

fig = figure('Name','Simulink Camera (LT)','NumberTitle','off',...
             'MenuBar','none','ToolBar','none',...
             'Position',[100 100 680 520]);

ax   = axes('Parent',fig);
h_raw = imshow(zeros(480,640,3,'uint8'),'Parent',ax);
title(ax,'Simulink Camera  (latest\_frame  —  what Pi is receiving)');

live_view_timer_LT = timer(...
    'Name',          'live_cam_viewer_LT',...
    'Period',        0.2,...
    'ExecutionMode', 'fixedRate',...
    'TimerFcn',      @(~,~) update_view_LT(h_raw, fig));

start(live_view_timer_LT);
disp('Live camera view (LT) started — 5 Hz, single panel. Close figure or stop(live_view_timer_LT) to stop.')

function update_view_LT(h_raw, fig)
    if ~ishandle(fig)
        t = timerfindall('Name','live_cam_viewer_LT');
        if ~isempty(t), stop(t); delete(t); end
        return;
    end
    try
        frame = evalin('base','latest_frame');
        if ~isempty(frame)
            set(h_raw,'CData',frame);
        end
    catch
    end
    drawnow limitrate;   % skips flush if GPU is already busy — no blocking
end
