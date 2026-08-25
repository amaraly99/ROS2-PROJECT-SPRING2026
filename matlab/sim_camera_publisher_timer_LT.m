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

% ── Stereo (optional) ──────────────────────────────────────────────────────
% Set by set_stereo.m, which flips the Simulink blocks at the same time.
if ~exist('STEREO_ON','var') || isempty(STEREO_ON), STEREO_ON = 0; end
if ~exist('cam_pub_right','var'), cam_pub_right = []; end
if ~exist('cam_msg_right','var'), cam_msg_right = []; end

if STEREO_ON
    if isempty(cam_pub_right) || isempty(cam_msg_right)
        error(['cam_pub_right missing. Re-run hil_ros_init_LT after set_stereo(1) ' ...
               'so the right-eye publisher gets created.']);
    end
    if ~exist('latest_frame_right','var')
        error(['latest_frame_right missing. Run set_stereo(1), then restart the ' ...
               'model so the right camera renders.']);
    end
else
    % Never publish a right eye when the model is not rendering one.
    cam_pub_right = [];
    cam_msg_right = [];
end

% Cross-check the MODEL against STEREO_ON. The documented startup sequence begins
% with `clear all`, which wipes STEREO_ON but NOT the commented/uncommented state
% of the right-eye blocks — so the two can drift apart and you would get a topic
% with no frames, or a camera rendering into nothing, with no error anywhere.
% By the time this script runs the model is loaded, so this is the right place to
% catch it, and it is a hard error rather than a silent mismatch.
try
    model_stereo = strcmp(get_param('hil_closed_loop/Simulation 3D Camera Right', ...
                                    'Commented'), 'off');
catch
    model_stereo = false;   % block absent => stereo was never added to the model
end
if model_stereo ~= logical(STEREO_ON)
    modes = {'MONO','STEREO'};
    error(['STEREO mismatch: the model is in %s mode but STEREO_ON = %d.\n' ...
           'Most likely `clear all` wiped STEREO_ON after you ran set_stereo.\n' ...
           'Fix: set_stereo(%d), then re-run hil_ros_init_LT, then re-run this script.'], ...
           modes{double(model_stereo)+1}, double(STEREO_ON), double(model_stereo));
end

old = timerfindall('Name','sim_cam_pub_timer_LT');
if ~isempty(old), stop(old); delete(old); end

assignin('base', 'sim_cam_pub_count_LT',    0);   % fresh frames sent
assignin('base', 'sim_cam_repeat_count_LT', 0);   % repeated frames sent
assignin('base', 'sim_cam_drop_count_LT',   0);   % nothing-to-send ticks
assignin('base', 'sim_cam_pub_count_right_LT', 0);% right-eye sends (stereo only)

sim_cam_pub_timer_LT = timer( ...
    'Name',          'sim_cam_pub_timer_LT', ...
    'Period',        1 / PUBLISH_HZ, ...
    'ExecutionMode', 'fixedRate', ...
    'BusyMode',      'drop', ...
    'TimerFcn',      @(~,~) publish_frame_LT(cam_pub, cam_msg, cam_pub_right, cam_msg_right), ...
    'ErrorFcn',      @(~,e) fprintf('[cam-LT-err] %s\n', e.message));

start(sim_cam_pub_timer_LT);
if STEREO_ON
    fprintf('sim_cam_pub_timer_LT started at %d Hz (constant-rate, STEREO)\n', PUBLISH_HZ);
    fprintf('  left:  /sim/camera/image_raw\n');
    fprintf('  right: /sim/camera/right/image_raw\n');
    fprintf('  ~37 MB/s total (~295 Mbit/s) — wired Ethernet only.\n');
else
    fprintf('sim_cam_pub_timer_LT started at %d Hz (constant-rate, MONO) on /sim/camera/image_raw\n', PUBLISH_HZ);
end
fprintf('  Stop:     stop(sim_cam_pub_timer_LT)\n');
fprintf('  Counters: sim_cam_pub_count_LT  sim_cam_repeat_count_LT  sim_cam_drop_count_LT\n');

function publish_frame_LT(pub, msg, pub_r, msg_r)
    % Idle auto-throttle config: when the scene has been static this long, drop
    % the real send rate to a heartbeat so we never blast 18 MB/s into a dead
    % loop (that sustained throughput with no consumer crashed MATLAB's ROS2
    % layer over a long idle period). HB_DIV=5 at a 20 Hz timer => ~4 Hz idle.
    %
    % THRESHOLD = 120 s, NOT a few seconds: during a run the drone sits STILL for
    % the ~35 s warmup while SLAM builds its first keyframes. Throttling the camera
    % then starves SLAM init -> the controller cannot approach -> the drone just
    % searches the whole run (and startup can fail). A run cycle is only ~95 s, so
    % 120 s never triggers inside a run; it only catches genuine multi-minute idle
    % (between chunks / abandoned session), which is the case that crashed MATLAB.
    IDLE_THROTTLE_SEC = 120;
    HB_DIV            = 5;

    % STEREO: driven purely by whether hil_ros_init_LT created a right-eye
    % publisher. In mono this is false and every stereo branch below is skipped,
    % so the mono path is byte-for-byte what it was before stereo existed.
    stereo = ~isempty(pub_r);

    persistent report_tic prev_total pub_count rep_count drop_count pdata have_good
    persistent last_cs uniq_count prev_uniq last_change_tic hb_tick idle_skip throttled
    persistent pdata_right pub_count_r
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
    if isempty(pdata_right), pdata_right = zeros(3,640*480,'uint8'); end
    if isempty(pub_count_r), pub_count_r = 0;                        end

    try
        frame = evalin('base', 'latest_frame');
        valid = ~isempty(frame) && isa(frame,'uint8') && ndims(frame) == 3 && ...
                size(frame,1) == 480 && size(frame,2) == 640 && size(frame,3) == 3;

        % Right eye, guarded: a missing variable costs one branch, not a
        % [cam-LT-err] spew every single tick.
        valid_r = false;
        frame_r = [];
        if stereo
            try
                frame_r = evalin('base', 'latest_frame_right');
            catch
                frame_r = [];
            end
            valid_r = ~isempty(frame_r) && isa(frame_r,'uint8') && ndims(frame_r) == 3 && ...
                      size(frame_r,1) == 480 && size(frame_r,2) == 640 && size(frame_r,3) == 3;
        end

        % PAIR ATOMICITY. In stereo a tick is only usable if BOTH eyes are good.
        % Never send a left frame without its matching right frame: an unpaired
        % frame breaks stereo triangulation outright, whereas a repeated pair is
        % merely stale. In mono this reduces to plain `valid`.
        pair_valid = valid && (~stereo || valid_r);

        sent = false;
        if valid
            % DIAGNOSTIC (does NOT drop frames): sparse checksum counts how many
            % sent frames are actually NEW, and resets the idle clock when the
            % scene changes. Keyed on the LEFT eye only, so mono and stereo
            % throttle at exactly the same moments.
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
            % In stereo this gates BOTH eyes together — the leak guard must not
            % be weakened by the pairing logic below.
            throttled = toc(last_change_tic) > IDLE_THROTTLE_SEC;
            hb_tick = hb_tick + 1;
            if throttled
                idle_skip = idle_skip + 1;   % static scene: send NOTHING (leak guard)
            elseif pair_valid
                % RGB->BGR + HxWxC col-major -> CxWxH row-major (per-channel transpose)
                pdata(1,:) = reshape(frame(:,:,3).', 1, []);
                pdata(2,:) = reshape(frame(:,:,2).', 1, []);
                pdata(3,:) = reshape(frame(:,:,1).', 1, []);
                if stereo
                    % Byte-identical packing to the left eye. Any difference here
                    % swaps red and blue in one eye only, which degrades stereo
                    % matching silently.
                    pdata_right(1,:) = reshape(frame_r(:,:,3).', 1, []);
                    pdata_right(2,:) = reshape(frame_r(:,:,2).', 1, []);
                    pdata_right(3,:) = reshape(frame_r(:,:,1).', 1, []);
                end
                have_good  = true;
                pub_count  = pub_count + 1;
                sent = true;
            elseif have_good
                % Stereo only: left eye is fresh but the right one is missing this
                % tick. Hold the last complete PAIR rather than break pairing.
                rep_count = rep_count + 1;
                sent = true;
            else
                drop_count = drop_count + 1;
            end
        elseif have_good
            % Sim paused/resetting and latest_frame went empty — hold the rate
            % by re-sending the cached last-good frame (the cached PAIR, in stereo).
            rep_count = rep_count + 1;
            sent = true;
        else
            % Never seen a valid frame yet (startup) — nothing to send.
            drop_count = drop_count + 1;
        end

        if sent
            % ONE clock read, applied to BOTH messages. This is the whole reason
            % the two eyes share a timestamp: sim_heartbeat is the SIMULATION
            % clock (written by the sim_clock_writer block inside Simulink), so
            % reading it once per tick makes the pair simultaneous by
            % construction. Reading it twice would reintroduce exactly the sync
            % problem stereo SLAM cannot tolerate.
            have_stamp = false;
            sec  = int32(0);
            nsec = uint32(0);
            try
                t_sim = double(evalin('base','sim_heartbeat'));
                sec   = int32(floor(t_sim));
                nsec  = uint32((t_sim - double(sec)) * 1e9);
                have_stamp = true;
            catch
            end

            msg.data = pdata(:);
            if have_stamp
                msg.header.stamp.sec     = sec;
                msg.header.stamp.nanosec = nsec;
            end
            send(pub, msg);

            if stereo
                msg_r.data = pdata_right(:);
                if have_stamp
                    msg_r.header.stamp.sec     = sec;
                    msg_r.header.stamp.nanosec = nsec;
                end
                send(pub_r, msg_r);
                pub_count_r = pub_count_r + 1;
            end
        end

        % Keep base-workspace counters in sync (other scripts read these).
        assignin('base','sim_cam_pub_count_LT',       pub_count);
        assignin('base','sim_cam_repeat_count_LT',    rep_count);
        assignin('base','sim_cam_drop_count_LT',      drop_count);
        assignin('base','sim_cam_pub_count_right_LT', pub_count_r);

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
            if stereo
                % The right eye must track the left exactly. A gap means send()
                % is failing on the right publisher — this replaces "ros2 topic
                % hz", which MATLAB's ros2 command does not support.
                eyes = sprintf('  L=%d R=%d', total, pub_count_r);
                if pub_count_r ~= total
                    eyes = [eyes '  <-- EYES OUT OF STEP'];
                end
            else
                eyes = '';
            end
            fprintf('[cam-LT] %.1f Hz sent, %.1f Hz NEW  (fresh=%d repeat=%d uniq=%d)%s%s\n', ...
                    rate, urate, pub_count, rep_count, uniq_count, eyes, flag);
            prev_total = total;
            prev_uniq  = uniq_count;
            report_tic = tic;
        end

    catch err
        fprintf('[cam-LT-err] %s\n', err.message);
    end
end
