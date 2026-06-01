% sim_camera_publisher_timer_LT.m  (Lightweight — rate-configurable)
% STEP 3 (LT): Run after Simulink is running and latest_frame is populating.
% Requires hil_ros_init or hil_ros_init_LT to have run first.
%
% LT features:
%   - Configurable publish rate (change PUBLISH_HZ below and re-run)
%   - Skips duplicate frames (sparse checksum) — no wasted DDS send on paused sim
%   - Prints count + live rate every 3 s — no terminal flood
%
% To STOP:    stop(sim_cam_pub_timer_LT)
% To RESTART at new rate: edit PUBLISH_HZ and re-run this script.

% ── Rate configuration ─────────────────────────────────────────────────────
% Test order: 14 → 20 → 30
%   14 Hz = 12.9 MB/s  — safe starting point
%   20 Hz = 18.4 MB/s  — target; test after Pi backward traffic confirmed off
%   30 Hz = 27.6 MB/s  — only try if 20 Hz shows zero sim lag
%   NOTE: Simulink fixed-step is 50 ms (20 Hz). Publishing above 20 Hz sends
%   duplicate frames — dedup skips them, so effective delivery stays at 20 Hz.
%   To actually get 30 Hz unique frames, Simulink step must also be reduced.
PUBLISH_HZ = 20;
% ───────────────────────────────────────────────────────────────────────────

if ~exist('cam_pub','var'),      error('cam_pub missing. Run hil_ros_init first.'); end
if ~exist('cam_msg','var'),      error('cam_msg missing. Run hil_ros_init first.'); end
if ~exist('latest_frame','var'), error('latest_frame missing. Run Simulink first.'); end

old = timerfindall('Name','sim_cam_pub_timer_LT');
if ~isempty(old), stop(old); delete(old); end

assignin('base', 'sim_cam_pub_count_LT',  0);
assignin('base', 'sim_cam_drop_count_LT', 0);

sim_cam_pub_timer_LT = timer( ...
    'Name',          'sim_cam_pub_timer_LT', ...
    'Period',        1 / PUBLISH_HZ, ...
    'ExecutionMode', 'fixedRate', ...
    'BusyMode',      'drop', ...
    'TimerFcn',      @(~,~) publish_frame_LT(cam_pub, cam_msg), ...
    'ErrorFcn',      @(~,e) fprintf('[cam-LT-err] %s\n', e.message));

start(sim_cam_pub_timer_LT);
fprintf('sim_cam_pub_timer_LT started at %d Hz on /sim/camera/image_raw\n', PUBLISH_HZ);
fprintf('  Stop:     stop(sim_cam_pub_timer_LT)\n');
fprintf('  Counters: sim_cam_pub_count_LT   sim_cam_drop_count_LT\n');

function publish_frame_LT(pub, msg)
    persistent last_cs report_tic prev_count pdata
    if isempty(last_cs),    last_cs    = uint32(0);               end
    if isempty(report_tic), report_tic = tic;                     end
    if isempty(prev_count), prev_count = int32(0);                end
    if isempty(pdata),      pdata      = zeros(3,640*480,'uint8'); end

    try
        frame = evalin('base', 'latest_frame');
        if isempty(frame) || ~isa(frame,'uint8') || ndims(frame) ~= 3 || ...
           size(frame,1) ~= 480 || size(frame,2) ~= 640 || size(frame,3) ~= 3
            d = evalin('base','sim_cam_drop_count_LT');
            assignin('base','sim_cam_drop_count_LT', d + 1);
        else
            % Sparse checksum (~1500 samples) — cheap duplicate-frame guard.
            % Skips the full BGR convert + DDS send when Simulink is paused.
            cs = sum(uint32(frame(1:200:end)));
            if cs == last_cs
                d = evalin('base','sim_cam_drop_count_LT');
                assignin('base','sim_cam_drop_count_LT', d + 1);
            else
                last_cs = cs;

                % RGB→BGR + H×W×C col-major → C×W×H row-major (per-channel 2-D transpose)
                pdata(1,:) = reshape(frame(:,:,3).', 1, []);
                pdata(2,:) = reshape(frame(:,:,2).', 1, []);
                pdata(3,:) = reshape(frame(:,:,1).', 1, []);
                msg.data = pdata(:);

                try
                    t_sim = double(evalin('base','sim_heartbeat'));
                    sec   = int32(floor(t_sim));
                    nsec  = uint32((t_sim - double(sec)) * 1e9);
                    msg.header.stamp.sec     = sec;
                    msg.header.stamp.nanosec = nsec;
                catch
                end

                send(pub, msg);
                c = evalin('base','sim_cam_pub_count_LT');
                assignin('base','sim_cam_pub_count_LT', c + 1);
            end
        end

        % Live rate report — prints once every 3 s, not every callback
        elapsed = toc(report_tic);
        if elapsed >= 3.0
            c = evalin('base','sim_cam_pub_count_LT');
            if c < prev_count, prev_count = int32(0); end   % counter was reset
            rate = double(c - prev_count) / elapsed;
            fprintf('[cam-LT] published: %d  (+%d in %.1fs = %.1f Hz)\n', ...
                    c, c - prev_count, elapsed, rate);
            prev_count = c;
            report_tic = tic;
        end

    catch err
        fprintf('[cam-LT-err] %s\n', err.message);
    end
end
