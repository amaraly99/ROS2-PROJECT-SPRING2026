% hil_publish_frame.m — camera publisher timer callback (one frame per tick).
% Moved out of sim_camera_publisher_timer_LT.m unchanged in behaviour, so that
% hil_pub_control('start') can reset its saved state with `clear hil_publish_frame`
% (persistent variables: last good frame, checksums, idle clock, counters).
% Called only by the timer that hil_pub_control creates.
function hil_publish_frame(pub, msg)
    % Idle auto-throttle config: when the scene has been static this long, drop
    % the real send rate to a heartbeat so we never blast 18 MB/s into a dead
    % loop (that sustained throughput with no consumer crashed MATLAB's ROS2
    % layer over a long idle period). Once throttled we send NOTHING rather than
    % a reduced-rate heartbeat, so there is no heartbeat divisor any more.
    %
    % THRESHOLD = 120 s, NOT a few seconds: during a run the drone sits STILL for
    % the ~35 s warmup while SLAM builds its first keyframes. Throttling the camera
    % then starves SLAM init -> the controller can't approach -> the drone just
    % searches the whole run (and startup can fail). A run cycle is only ~95 s, so
    % 120 s never triggers inside a run; it only catches genuine multi-minute idle
    % (between chunks / abandoned session), which is the case that crashed MATLAB.
    IDLE_THROTTLE_SEC = 120;
    % STALE GUARDS (visible freeze instead of a silent one):
    %   REPEAT_MAX_SEC — latest_frame empty/invalid: re-send the cached frame for at
    %     most this long (covers a normal supervisor restart), then send NOTHING so
    %     the Pi's wait_for_frames fails loudly instead of recording a frozen run.
    %     Set to Inf to restore the old unlimited re-send.
    %   STALE_WARN_SEC — valid frames but identical content this long -> warn.
    REPEAT_MAX_SEC    = 15;
    STALE_WARN_SEC    = 5;
    WARN_EVERY_SEC    = 3;

    persistent report_tic prev_total pub_count rep_count drop_count pdata have_good
    persistent last_cs uniq_count prev_uniq last_change_tic hb_tick idle_skip throttled
    persistent last_good_tic warn_tic armed baseline_cs hold_tic
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
    if isempty(last_good_tic), last_good_tic = tic;                end
    if isempty(warn_tic),   warn_tic   = tic;                      end
    if isempty(armed)       % first tick after hil_pub_control('start'): hold until the image changes
        armed = true; hold_tic = tic;
        baseline_cs = [];
        try, baseline_cs = evalin('base', 'sim_cam_pub_baseline_cs'); catch, end
    end

    try
        frame = evalin('base', 'latest_frame');
        valid = ~isempty(frame) && isa(frame,'uint8') && ndims(frame) == 3 && ...
                size(frame,1) == 480 && size(frame,2) == 640 && size(frame,3) == 3;

        sent = false;
        if valid && armed
            % Hold: do not publish the image that was already in latest_frame at pub_start.
            if ~isempty(baseline_cs) && sum(uint32(frame(1:200:end))) == baseline_cs
                valid = false;          % falls through to the existing 'nothing to send' branch (counted as drop there)
                if toc(warn_tic) > WARN_EVERY_SEC
                    fprintf(2, '[cam-LT] holding: latest_frame unchanged since pub_start (%.0f s) — sim not restarted yet?\n', toc(hold_tic));
                    warn_tic = tic;
                end
            else
                armed = false;
                fprintf('[cam-LT] first new frame %.1f s after pub_start — publishing\n', toc(hold_tic));
            end
        end
        if valid
            last_good_tic = tic;
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
            if ~throttled && toc(last_change_tic) > STALE_WARN_SEC && toc(warn_tic) > WARN_EVERY_SEC
                fprintf(2, '[cam-LT] WARNING: frames valid but IDENTICAL for %.0f s — sim frozen? (3D window closed?)\n', ...
                        toc(last_change_tic));
                warn_tic = tic;
            end
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
            % Sim paused/resetting and latest_frame went empty. Re-send the cached
            % last-good frame to hold the rate, but only for REPEAT_MAX_SEC, and
            % say so while it happens.
            stale_s = toc(last_good_tic);
            if stale_s <= REPEAT_MAX_SEC
                rep_count = rep_count + 1;
                sent = true;
            else
                drop_count = drop_count + 1;
            end
            if toc(warn_tic) > WARN_EVERY_SEC
                if sent
                    fprintf(2, '[cam-LT] WARNING: latest_frame empty %.1f s — RE-SENDING STALE FRAME (stops at %g s)\n', ...
                            stale_s, REPEAT_MAX_SEC);
                else
                    fprintf(2, '[cam-LT] WARNING: latest_frame empty %.0f s — STOPPED SENDING (sim not producing frames)\n', ...
                            stale_s);
                end
                warn_tic = tic;
            end
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
