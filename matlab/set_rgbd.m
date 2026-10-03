function set_rgbd(on)
%SET_RGBD  Select the RGB-D model (hil_closed_loop_baseline_RGBD) and flag the
%          MATLAB scripts to publish left-camera depth.
%
%   set_rgbd(1)   RGB-D: checks the RGB-D model is wired for left depth, sets
%                 RGBD_ON=1 and STEREO_ON=0 in the base workspace.
%   set_rgbd(0)   clears RGBD_ON and any leftover depth frame.
%
% This is the RGB-D twin of set_stereo.m, but it does NOT toggle blocks.
% RGB-D lives in its own model file (hil_closed_loop_baseline_RGBD.slx), so the
% mono/stereo model hil_closed_loop.slx is never touched. set_stereo.m must not
% be called in an RGB-D session: it hardcodes MODEL='hil_closed_loop'.
%
% AFTER set_rgbd(1)
%   1. run hil_ros_init_LT_rgbd             (mono init + depth publisher)
%   2. start hil_closed_loop_baseline_RGBD  (not hil_closed_loop)
%   3. run sim_camera_publisher_timer_LT_rgbd
%
% ⚠ Plant/camera fixes made to hil_closed_loop.slx must be copied into the
%   RGB-D model by hand. The two files do not share blocks.
%
% See also HIL_ROS_INIT_LT_RGBD, SIM_CAMERA_PUBLISHER_TIMER_LT_RGBD.

    MODEL = 'hil_closed_loop_baseline_RGBD';
    CAM   = [MODEL '/Simulation 3D Camera'];
    DEPTH = [MODEL '/write_depth_left'];

    if nargin < 1
        error('set_rgbd: pass 1 for RGB-D or 0 to clear.');
    end
    on = logical(on);

    if ~on
        assignin('base','RGBD_ON', 0);
        evalin('base', 'clear latest_depth_left');
        fprintf('set_rgbd: RGB-D OFF.\n');
        return
    end

    load_system(MODEL);

    % The model must be exactly the one the depth test verified (HANDOFF A2b).
    % A mismatch here would publish colour with no depth, and RTAB-Map's exact
    % sync would silently drop every frame.
    problems = {};
    if ~strcmp(get_param(CAM,'Commented'),'off'),           problems{end+1} = 'left camera is commented out'; end
    if ~strcmp(get_param(CAM,'DepthOutportEnabled'),'on'),  problems{end+1} = 'left camera depth port is off'; end
    try
        if ~strcmp(get_param(DEPTH,'Commented'),'off'),     problems{end+1} = 'write_depth_left is commented out'; end
    catch
        problems{end+1} = 'write_depth_left block not found';
    end
    fl = str2num(get_param(CAM,'FocalLength')); %#ok<ST2NM>
    sz = str2num(get_param(CAM,'ImageSize'));   %#ok<ST2NM>
    if ~isequal(fl,[554 554]), problems{end+1} = sprintf('FocalLength=%s, Pi expects [554 554]', mat2str(fl)); end
    if ~isequal(sz,[480 640]), problems{end+1} = sprintf('ImageSize=%s, publisher expects [480 640]', mat2str(sz)); end
    % Right eye must stay off: depth comes from the LEFT camera only (Q6b).
    rb = find_system(MODEL,'SearchDepth',1,'Regexp','on','Name','Right|_right$');
    for k = 1:numel(rb)
        if strcmp(get_param(rb{k},'Commented'),'off')
            problems{end+1} = sprintf('%s is active (must be commented)', rb{k}); %#ok<AGROW>
        end
    end
    if ~isempty(problems)
        error('set_rgbd: %s is not in the verified RGB-D state:\n  - %s', MODEL, strjoin(problems, sprintf('\n  - ')));
    end

    if bdIsLoaded('hil_closed_loop')
        warning('set_rgbd:monoLoaded', ...
            'hil_closed_loop is also loaded. Start only %s, or two sims will fight over the scene.', MODEL);
    end

    assignin('base','RGBD_ON',   1);
    assignin('base','STEREO_ON', 0);
    assignin('base','HIL_MODEL', MODEL);
    evalin('base', 'clear latest_depth_left latest_frame_right latest_depth_right');

    fprintf('set_rgbd: RGB-D ON, model %s (left depth -> /sim/camera/depth/image_raw).\n', MODEL);
    fprintf('  Next: run hil_ros_init_LT_rgbd, start %s, run sim_camera_publisher_timer_LT_rgbd.\n', MODEL);
end
