% sim_camera_publisher_timer_LT.m  (Lightweight — constant-rate, bulletproof)
% STEP 3 (LT): Run after Simulink is running and latest_frame is populating.
% Requires hil_ros_init or hil_ros_init_LT to have run first.
%
% CONSTANT-RATE DESIGN (changed from the old dedup version):
%   The detector benchmark needs a STEADY camera feed so every detector config
%   sees identical input conditions. The old build deduplicated frames (skipped
%   any frame identical to the previous one), which dropped the effective rate
%   to whatever the sim produced (9-13 Hz when Simulink runs below real-time)
%   and made the rate jitter run-to-run. That is bad for a benchmark and it also
%   let the Pi's "wait_for_frames" time out during a sim reset.
%
%   This version publishes at a FIXED rate no matter what:
%     - valid new frame      -> convert + cache + send  (counted "fresh")
%     - invalid/empty frame  -> re-send the last good frame (counted "repeat")
%   So the Pi always sees a continuous PUBLISH_HZ stream, even while the
%   supervisor is stopping/restarting the model between runs.
%
% To STOP:    stop(sim_cam_pub_timer_LT)
% To change rate: edit PUBLISH_HZ below and re-run this script.

% ── Rate configuration ─────────────────────────────────────────────────────
%   Simulink fixed-step is 50 ms (20 Hz). 20 Hz = 18.4 MB/s (already confirmed
%   safe). Going above 20 Hz only repeats frames faster — it does not create
%   new unique frames unless the Simulink step is also reduced.
PUBLISH_HZ = 20;
% ───────────────────────────────────────────────────────────────────────────

if ~exist('cam_pub','var'),      error('cam_pub missing. Run hil_ros_init first.'); end
if ~exist('cam_msg','var'),      error('cam_msg missing. Run hil_ros_init first.'); end
if ~exist('latest_frame','var'), error('latest_frame missing. Run Simulink first.'); end

old = timerfindall('Name','sim_cam_pub_timer_LT');
if ~isempty(old), stop(old); delete(old); end

assignin('base', 'sim_cam_pub_count_LT',    0);   % fresh frames sent
assignin('base', 'sim_cam_repeat_count_LT', 0);   % repeated frames sent
assignin('base', 'sim_cam_drop_count_LT',   0);   % nothing-to-send ticks

sim_cam_pub_timer_LT = timer( ...
    'Name',          'sim_cam_pub_timer_LT', ...
    'Period',        1 / PUBLISH_HZ, ...
    'ExecutionMode', 'fixedRate', ...
    'BusyMode',      'drop', ...
    'TimerFcn',      @(~,~) publish_frame_LT(cam_pub, cam_msg), ...
    'ErrorFcn',      @(~,e) fprintf('[cam-LT-err] %s\n', e.message));

start(sim_cam_pub_timer_LT);
fprintf('sim_cam_pub_timer_LT started at %d Hz (constant-rate) on /sim/camera/image_raw\n', PUBLISH_HZ);
fprintf('  Stop:     stop(sim_cam_pub_timer_LT)\n');
fprintf('  Counters: sim_cam_pub_count_LT  sim_cam_repeat_count_LT  sim_cam_drop_count_LT\n');

function publish_frame_LT(pub, msg)
    % Idle auto-throttle config: when the scene has been static this long, drop
    % the real send rate to a heartbeat so we never blast 18 MB/s into a dead
    % loop (that sustained throughput with no consumer crashed MATLAB's ROS2
    % layer over a long idle period). HB_DIV=5 at a 20 Hz timer => ~4 Hz idle.
    %
    % THRESHOLD = 120 s, NOT a few seconds: during a run the drone sits STILL for
    % the ~35 s warmup while SLAM builds its first keyframes. Throttling the camera
    % then starves SLAM init -> the controller can't approach -> the drone just
    % searches the whole run (and startup can fail). A run cycle is only ~95 s, so
    % 120 s never triggers inside a run; it only catches genuine multi-minute idle
    % (between chunks / abandoned session), which is the case that crashed MATLAB.
    IDLE_THROTTLE_SEC = 120;
    HB_DIV            = 5;

    persistent report_tic prev_total pub_count rep_count drop_count pdata have_good
    persistent last_cs uniq_count prev_uniq last_change_tic hb_tick idle_skip throttled
    if isempty(report_tic), report_tic = tic;                      end
    if isempty(prev_total), prev_total = 0;                        end
    if isempty(pub_count),  pub_count  = 0;                        end
    if isempty(rep_count),  rep_count  = 0;                        end
    if isempty(drop_count), drop_count = 0;                        end
    if isempty(pdata),      pdata      = zeros(3,640*480,'uint8'); end
    if isempty(have_good),  have_good  = false;                    end
    if isempty(last_cs),    last_cs    = uint32(0);                end
    if isempty(uniq_count), uniq_count = 0;                        end
    if isempty(prev_uniq),  prev_uniq  = 0;                        end
    if isempty(last_change_tic), last_change_tic = tic;            end
    if isempty(hb_tick),    hb_tick    = 0;                        end
    if isempty(idle_skip),  idle_skip  = 0;                        end
    if isempty(throttled),  throttled  = false;                    end

    try
        frame = evalin('base', 'latest_frame');
        valid = ~isempty(frame) && isa(frame,'uint8') && ndims(frame) == 3 && ...
                size(frame,1) == 480 && size(frame,2) == 640 && size(frame,3) == 3;

        sent = false;
        if valid
            % DIAGNOSTIC (does NOT drop frames): sparse checksum counts how many
            % sent frames are actually NEW, and resets the idle clock when the
            % scene changes.
            cs = sum(uint32(frame(1:200:end)));
            if cs ~= last_cs
                uniq_count = uniq_count + 1;
                last_cs = cs;
                last_change_tic = tic;     % scene moved -> not idle
            end

            % Throttle only when the scene has been static for IDLE_THROTTLE_SEC.
            % During a live run (frames changing) this never engages, so the Pi
            % still sees a steady ~20 Hz. When it DOES engage we stop the heavy
            % convert+send ENTIRELY (not just slow it) — a 4 Hz trickle of 921 KB
            % messages still grew MATLAB's ROS2 layer to OUT OF MEMORY over a long
            % idle. The cheap checksum above keeps running, so the instant the
            % scene moves again (e.g. a sim restart) we resume at full rate.
            throttled = toc(last_change_tic) > IDLE_THROTTLE_SEC;
            hb_tick = hb_tick + 1;
            if throttled
                idle_skip = idle_skip + 1;   % static scene: send NOTHING (leak guard)
            else
                % RGB->BGR + H×W×C col-major -> C×W×H row-major (per-channel transpose)
                pdata(1,:) = reshape(frame(:,:,3).', 1, []);
                pdata(2,:) = reshape(frame(:,:,2).', 1, []);
                pdata(3,:) = reshape(frame(:,:,1).', 1, []);
                have_good  = true;
                pub_count  = pub_count + 1;
                sent = true;
            end
        elseif have_good
            % Sim paused/resetting and latest_frame went empty — hold the rate
            % by re-sending the cached last-good frame.
            rep_count = rep_count + 1;
            sent = true;
        else
            % Never seen a valid frame yet (startup) — nothing to send.
            drop_count = drop_count + 1;
        end

        if sent
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
        end

        % Keep base-workspace counters in sync (other scripts read these).
        assignin('base','sim_cam_pub_count_LT',    pub_count);
        assignin('base','sim_cam_repeat_count_LT', rep_count);
        assignin('base','sim_cam_drop_count_LT',   drop_count);

        % Live rate report every 3 s.
        elapsed = toc(report_tic);
        if elapsed >= 3.0
            total = pub_count + rep_count;
            if total < prev_total, prev_total = 0; end   % counters were reset
            if uniq_count < prev_uniq, prev_uniq = 0; end
            rate  = double(total - prev_total) / elapsed;
            urate = double(uniq_count - prev_uniq) / elapsed;
            flag  = '';
            if throttled
                flag = '  [IDLE: sending paused (static scene) — resumes on motion]';
            elseif urate < 1.0
                flag = '  <-- no new frames (about to throttle)';
            end
            fprintf('[cam-LT] %.1f Hz sent, %.1f Hz NEW  (fresh=%d repeat=%d uniq=%d)%s\n', ...
                    rate, urate, pub_count, rep_count, uniq_count, flag);
            prev_total = total;
            prev_uniq  = uniq_count;
            report_tic = tic;
        end

    catch err
        fprintf('[cam-LT-err] %s\n', err.message);
    end
end
