% sim_camera_publisher_timer_LT.m  (Lightweight — rate-configurable, stereo-capable)
% STEP 3 (LT): Run after Simulink is running and latest_frame is populating.
% Requires hil_ros_init or hil_ros_init_LT to have run first.
%
% LT features:
%   - Configurable publish rate (change PUBLISH_HZ below and re-run)
%   - Skips duplicate frames (sparse checksum) — no wasted DDS send on paused sim
%   - Prints count + live rate every 3 s — no terminal flood
%   - Stereo (optional): publishes a paired right eye on ONE shared timestamp,
%     set by set_stereo.m — see the STEREO block below.
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
%   Stereo roughly doubles the bytes/s at the same PUBLISH_HZ — wired Ethernet only.
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

assignin('base', 'sim_cam_pub_count_LT',       0);
assignin('base', 'sim_cam_drop_count_LT',      0);
assignin('base', 'sim_cam_pub_count_right_LT', 0);   % right-eye sends (stereo only)

sim_cam_pub_timer_LT = timer( ...
    'Name',          'sim_cam_pub_timer_LT', ...
    'Period',        1 / PUBLISH_HZ, ...
    'ExecutionMode', 'fixedRate', ...
    'BusyMode',      'drop', ...
    'TimerFcn',      @(~,~) publish_frame_LT(cam_pub, cam_msg, cam_pub_right, cam_msg_right), ...
    'ErrorFcn',      @(~,e) fprintf('[cam-LT-err] %s\n', e.message));

start(sim_cam_pub_timer_LT);
if STEREO_ON
    fprintf('sim_cam_pub_timer_LT started at %d Hz, STEREO\n', PUBLISH_HZ);
    fprintf('  left:  /sim/camera/image_raw\n');
    fprintf('  right: /sim/camera/right/image_raw\n');
    fprintf('  Wired Ethernet only — stereo roughly doubles the bytes/s.\n');
else
    fprintf('sim_cam_pub_timer_LT started at %d Hz, MONO on /sim/camera/image_raw\n', PUBLISH_HZ);
end
fprintf('  Stop:     stop(sim_cam_pub_timer_LT)\n');
fprintf('  Counters: sim_cam_pub_count_LT   sim_cam_drop_count_LT   sim_cam_pub_count_right_LT\n');

function publish_frame_LT(pub, msg, pub_r, msg_r)
    % STEREO: driven purely by whether hil_ros_init_LT created a right-eye
    % publisher. In mono this is false and every stereo branch below is
    % skipped, so the mono path behaves exactly as it did before stereo existed.
    stereo = ~isempty(pub_r);

    persistent last_cs report_tic prev_count pdata pdata_right
    if isempty(last_cs),     last_cs     = uint32(0);               end
    if isempty(report_tic),  report_tic  = tic;                     end
    if isempty(prev_count),  prev_count  = int32(0);                end
    if isempty(pdata),       pdata       = zeros(3,640*480,'uint8'); end
    if isempty(pdata_right), pdata_right = zeros(3,640*480,'uint8'); end

    try
        frame = evalin('base', 'latest_frame');
        valid = ~isempty(frame) && isa(frame,'uint8') && ndims(frame) == 3 && ...
                size(frame,1) == 480 && size(frame,2) == 640 && size(frame,3) == 3;

        % Right eye, guarded: a missing variable costs one branch, not a
        % [cam-LT-err] spew every single tick.
        frame_r = [];
        valid_r = false;
        if stereo
            try
                frame_r = evalin('base', 'latest_frame_right');
            catch
                frame_r = [];
            end
            valid_r = ~isempty(frame_r) && isa(frame_r,'uint8') && ndims(frame_r) == 3 && ...
                      size(frame_r,1) == 480 && size(frame_r,2) == 640 && size(frame_r,3) == 3;
        end

        % PAIR ATOMICITY. In stereo a tick is only usable if BOTH eyes are
        % good this tick — never send a left frame without its matching right
        % frame, since an unpaired frame breaks stereo triangulation outright.
        % Unlike the old constant-rate design, this build has no repeat/hold
        % semantics at all (CB simplified that away for mono) — so an
        % unpaired stereo tick is simply dropped, same as an invalid one, for
        % the same reason mono drops rather than fabricates a frame.
        pair_valid = valid && (~stereo || valid_r);

        if ~pair_valid
            d = evalin('base','sim_cam_drop_count_LT');
            assignin('base','sim_cam_drop_count_LT', d + 1);
        else
            % Sparse checksum (~1500 samples) — cheap duplicate-frame guard.
            % Keyed on the LEFT eye only, so mono and stereo throttle at
            % exactly the same moments. Skips the full BGR convert + DDS send
            % on BOTH eyes together when Simulink is paused.
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

                if stereo
                    % Byte-identical packing to the left eye. Any difference
                    % here swaps red and blue in one eye only, which degrades
                    % stereo matching silently.
                    pdata_right(1,:) = reshape(frame_r(:,:,3).', 1, []);
                    pdata_right(2,:) = reshape(frame_r(:,:,2).', 1, []);
                    pdata_right(3,:) = reshape(frame_r(:,:,1).', 1, []);
                    msg_r.data = pdata_right(:);
                end

                % ONE clock read, applied to BOTH messages. sim_heartbeat is
                % the SIMULATION clock (sim_clock_writer inside Simulink), so
                % reading it once per tick makes the pair simultaneous by
                % construction. Reading it twice would reintroduce exactly
                % the sync problem stereo SLAM cannot tolerate.
                try
                    t_sim = double(evalin('base','sim_heartbeat'));
                    sec   = int32(floor(t_sim));
                    nsec  = uint32((t_sim - double(sec)) * 1e9);
                    msg.header.stamp.sec     = sec;
                    msg.header.stamp.nanosec = nsec;
                    if stereo
                        msg_r.header.stamp.sec     = sec;
                        msg_r.header.stamp.nanosec = nsec;
                    end
                catch
                end

                send(pub, msg);
                c = evalin('base','sim_cam_pub_count_LT');
                assignin('base','sim_cam_pub_count_LT', c + 1);

                if stereo
                    send(pub_r, msg_r);
                    cr = evalin('base','sim_cam_pub_count_right_LT');
                    assignin('base','sim_cam_pub_count_right_LT', cr + 1);
                end
            end
        end

        % Live rate report — prints once every 3 s, not every callback
        elapsed = toc(report_tic);
        if elapsed >= 3.0
            c = evalin('base','sim_cam_pub_count_LT');
            if c < prev_count, prev_count = int32(0); end   % counter was reset
            rate = double(c - prev_count) / elapsed;
            if stereo
                % The right eye must track the left exactly. A gap means
                % send() is failing on the right publisher — this replaces
                % "ros2 topic hz", which MATLAB's ros2 command does not support.
                cr = evalin('base','sim_cam_pub_count_right_LT');
                eyes = sprintf('  L=%d R=%d', c, cr);
                if cr ~= c
                    eyes = [eyes '  <-- EYES OUT OF STEP'];
                end
            else
                eyes = '';
            end
            fprintf('[cam-LT] published: %d  (+%d in %.1fs = %.1f Hz)%s\n', ...
                    c, c - prev_count, elapsed, rate, eyes);
            prev_count = c;
            report_tic = tic;
        end

    catch err
        fprintf('[cam-LT-err] %s\n', err.message);
    end
end
