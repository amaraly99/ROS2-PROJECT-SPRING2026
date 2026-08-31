function set_stereo(on)
%SET_STEREO  Switch hil_closed_loop between mono (0) and stereo (1).
%
%   set_stereo(0)   mono   — only the left camera renders and publishes.
%                            Byte-for-byte the configuration that produced
%                            every published result.
%   set_stereo(1)   stereo — both cameras render; both eyes publish, paired,
%                            sharing one simulation timestamp.
%
% WHY ONE COMMAND DOES BOTH HALVES
%   Stereo mode lives in two places that must always agree: the Simulink model
%   (does the second camera render?) and the MATLAB scripts (does the second
%   publisher exist?). If they ever disagree you get a topic with no frames, or
%   frames with nowhere to go, and the failure is silent. This function flips
%   both together so that cannot happen.
%
%   The model side uses Simulink's "Comment Out" on the right-eye blocks. A
%   commented block is excluded at COMPILE time, not merely skipped at run
%   time, so in mono mode the second camera costs literally nothing to render
%   and your existing mono timing results stay directly comparable. Commented
%   blocks are drawn greyed out and dashed, so the current mode is visible at a
%   glance in the model window.
%
% AFTER CALLING THIS
%   1. re-run  hil_ros_init_LT          (creates or destroys the right publisher)
%   2. restart the model                (Stop then Run)
%   3. re-run  sim_camera_publisher_timer_LT
%
%   The model runs in Accelerator mode, so the first run after a toggle rebuilds
%   the accelerator target (1-2 minutes). It has not hung.
%
% See also CHECK_STEREO_GEOMETRY, LIVE_CAMERA_VIEW_STEREO_LT.

    MODEL = 'hil_closed_loop';

    % The right-eye blocks added in Step 2 of the stereo walkthrough.
    % write_depth_right is optional (only present if you enabled the camera's
    % Ground Truth -> Output depth port), so a missing one is not an error.
    BLOCKS   = {'Simulation 3D Camera Right', 'write_frame_right', 'write_depth_right'};
    OPTIONAL = [false, false, true];

    if nargin < 1
        error('set_stereo: pass 0 for mono or 1 for stereo.');
    end
    on = logical(on);

    load_system(MODEL);

    % 'Commented' = 'on'  means the block is commented OUT, i.e. inactive.
    commented = 'on';
    if on, commented = 'off'; end

    missing = {};
    for k = 1:numel(BLOCKS)
        blk = [MODEL '/' BLOCKS{k}];
        try
            set_param(blk, 'Commented', commented);
        catch
            if ~OPTIONAL(k)
                missing{end+1} = BLOCKS{k}; %#ok<AGROW>
            end
        end
    end

    if ~isempty(missing)
        error(['set_stereo: block(s) not found in %s: %s\n' ...
               'Add the right-eye camera first (Step 2 of the stereo walkthrough), ' ...
               'and check the block names match exactly.'], ...
               MODEL, strjoin(missing, ', '));
    end

    assignin('base','STEREO_ON', double(on));

    if ~on
        % Drop any right-eye data left over from a previous stereo session, so
        % the publisher can never send a stale frame it mistakes for a live one.
        evalin('base', 'clear latest_frame_right latest_depth_right');
    end

    if on
        fprintf('set_stereo: STEREO — right camera active, /sim/camera/right/image_raw will publish.\n');
        fprintf('  Bandwidth doubles to ~37 MB/s (~295 Mbit/s). Wired Ethernet only.\n');
    else
        fprintf('set_stereo: MONO — right camera commented out, costs nothing to render.\n');
    end
    fprintf('  Next: re-run hil_ros_init_LT, restart the model, re-run sim_camera_publisher_timer_LT.\n');
end
