function out = hil_pub_control(action, hz)
% hil_pub_control — start / stop / query the camera publisher (per-run helper).
%   hil_pub_control('start', HZ)   stop any publisher timer, reset the callback's
%                                  saved state, create + start a new timer at HZ;
%                                  nothing is sent until latest_frame differs from
%                                  the image it held at this moment
%   hil_pub_control('stop')        stop + delete every publisher timer
%   s = hil_pub_control('status')  struct: running, ntimers, hz, fresh, repeat, drop
% Non-blocking, so it is safe to call from the supervisor's poll timer.
% Uses cam_pub / cam_msg from the base workspace (created once per session by
% hil_ros_init_LT). No ROS object is created or destroyed here: the only thing a
% start creates and a stop deletes is the timer, so nothing accumulates.
    NAME = 'sim_cam_pub_timer_LT';
    out = [];
    switch action
        case 'start'
            if nargin < 2 || isempty(hz), hz = 20; end
            hil_pub_control('stop');
            clear hil_publish_frame          % reset persistent state: the first frame sent is never an old one
            cam_pub = evalin('base', 'cam_pub');
            cam_msg = evalin('base', 'cam_msg');
            assignin('base', 'sim_cam_pub_count_LT',    0);
            assignin('base', 'sim_cam_repeat_count_LT', 0);
            assignin('base', 'sim_cam_drop_count_LT',   0);
            assignin('base', 'sim_cam_pub_hz',          hz);
            % Remember what latest_frame holds right now (e.g. the stopped sim's last image).
            % hil_publish_frame sends nothing until the image differs from it, so the first
            % frame published after a start is always a new one (never the pre-restart image).
            cs0 = [];
            try
                f0 = evalin('base', 'latest_frame');
                if ~isempty(f0) && isa(f0, 'uint8') && ndims(f0) == 3
                    cs0 = sum(uint32(f0(1:200:end)));
                end
            catch
            end
            assignin('base', 'sim_cam_pub_baseline_cs', cs0);
            t = timer( ...
                'Name',          NAME, ...
                'Period',        1 / hz, ...
                'ExecutionMode', 'fixedRate', ...
                'BusyMode',      'drop', ...
                'TimerFcn',      @(~,~) hil_publish_frame(cam_pub, cam_msg), ...
                'ErrorFcn',      @(~,e) fprintf('[cam-LT-err] %s\n', e.message));
            assignin('base', 'sim_cam_pub_timer_LT', t);   % same name as before: stop(sim_cam_pub_timer_LT) still works
            start(t);
            fprintf('[pub] started at %g Hz (callback state reset)\n', hz);
        case 'stop'
            old = timerfindall('Name', NAME);
            if ~isempty(old)
                stop(old);
                delete(old);
                fprintf('[pub] stopped, %d timer(s) deleted\n', numel(old));
            end
            if evalin('base', "exist('sim_cam_pub_timer_LT','var')")
                evalin('base', 'clear sim_cam_pub_timer_LT');
            end
        case 'status'
            t = timerfindall('Name', NAME);
            out.running = ~isempty(t) && strcmp(t(1).Running, 'on');
            out.ntimers = numel(t);
            out.hz      = base_or('sim_cam_pub_hz', NaN);
            out.fresh   = base_or('sim_cam_pub_count_LT', NaN);
            out.repeat  = base_or('sim_cam_repeat_count_LT', NaN);
            out.drop    = base_or('sim_cam_drop_count_LT', NaN);
        otherwise
            error('hil_pub_control: unknown action "%s"', action);
    end
end

function v = base_or(name, default)
    v = default;
    try
        if evalin('base', sprintf("exist('%s','var')", name))
            v = evalin('base', name);
        end
    catch
    end
end
