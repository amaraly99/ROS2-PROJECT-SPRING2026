% sim_camera_publisher_timer_LT_rgbd.m  (RGB-D copy of sim_camera_publisher_timer_LT.m, 2026-09-27)
% STEP 4 (RGB-D): run after hil_closed_loop_baseline_RGBD is running and
% latest_frame + latest_depth_left are populating. Requires hil_ros_init_LT_rgbd.
%
% Differences from sim_camera_publisher_timer_LT.m (which is untouched):
%   - no stereo branch (RGB-D uses the left camera only);
%   - every colour frame is sent together with its depth frame, both stamped
%     from ONE sim_heartbeat read. RTAB-Map pairs them with exact sync
%     (approx_sync=False in hil_rgbd.launch.py), so a 1 ns difference = dropped;
%   - a tick with colour but no valid depth is dropped, never sent half.
% Same timer NAME and counters as the mono script, so run_matrix's
% "timer is running" check and `stop(sim_cam_pub_timer_LT)` still work.
%
% To STOP: stop(sim_cam_pub_timer_LT)

PUBLISH_HZ = 20;   % Simulink fixed step is 1/20 s, so this is one send per sim step
% Optional override via environment (survives `clear all`), for link-capacity tests:
% 100 Mbps Ethernet carries ~11.9 MB/s, i.e. ~7 RGB-D frames/s (HANDOFF A9).
if ~isempty(getenv('RGBD_PUBLISH_HZ')), PUBLISH_HZ = str2double(getenv('RGBD_PUBLISH_HZ')); end
% Optional: send the colour image as mono8 (307 KB instead of 921 KB) when env
% RGBD_COLOUR_MONO=1. The Pi turns /sim/camera/image_raw into a gray /ovcam/image_raw
% anyway (sim_camera_bridge -> NV12 -> ovcam_bridge mono8), and sim_camera_bridge
% accepts any encoding via toCvCopy(msg,"rgb8") (sim_camera_bridge_node.cpp:134).
% Cuts RGB-D from ~1.54 to ~0.92 MB per tick so 20 Hz fits Yasser5G (HANDOFF A13).
COLOUR_MONO = strcmp(getenv('RGBD_COLOUR_MONO'), '1');
if COLOUR_MONO && exist('cam_msg','var')
    cam_msg.encoding = 'mono8';
    cam_msg.step     = uint32(640);
end

if ~exist('cam_pub','var'),           error('cam_pub missing. Run hil_ros_init_LT_rgbd first.'); end
if ~exist('depth_pub','var'),         error('depth_pub missing. Run hil_ros_init_LT_rgbd (not hil_ros_init_LT).'); end
if ~exist('latest_frame','var'),      error('latest_frame missing. Start hil_closed_loop_baseline_RGBD first.'); end
if ~exist('latest_depth_left','var'), error('latest_depth_left missing. Is hil_closed_loop_baseline_RGBD (not hil_closed_loop) running?'); end
if ~exist('RGBD_ON','var') || ~RGBD_ON, error('RGBD_ON not set. Run set_rgbd(1).'); end

old = timerfindall('Name','sim_cam_pub_timer_LT');
if ~isempty(old), stop(old); delete(old); end

assignin('base', 'sim_cam_pub_count_LT',       0);
assignin('base', 'sim_cam_drop_count_LT',      0);
assignin('base', 'sim_cam_pub_count_depth_LT', 0);
assignin('base', 'sim_cam_nodepth_count_LT',   0);   % ticks dropped because depth was invalid
assignin('base', 'sim_cam_dupstamp_count_LT',  0);   % ticks whose stamp repeated the previous send
sim_cam_pub_timer_LT = timer( ...
    'Name',          'sim_cam_pub_timer_LT', ...
    'Period',        1 / PUBLISH_HZ, ...
    'ExecutionMode', 'fixedRate', ...
    'BusyMode',      'drop', ...
    'TimerFcn',      @(~,~) publish_frame_LT_rgbd(cam_pub, cam_msg, depth_pub, depth_msg), ...
    'ErrorFcn',      @(~,e) fprintf('[cam-LT-err] %s\n', e.message));

start(sim_cam_pub_timer_LT);
fprintf('sim_cam_pub_timer_LT started at %d Hz, RGB-D, colour encoding %s\n', PUBLISH_HZ, cam_msg.encoding);
fprintf('  colour: /sim/camera/image_raw\n');
fprintf('  depth:  /sim/camera/depth/image_raw  (16UC1 mm, same stamp)\n');
fprintf('  Stop:     stop(sim_cam_pub_timer_LT)\n');
fprintf('  Counters: sim_cam_pub_count_LT  sim_cam_pub_count_depth_LT  sim_cam_drop_count_LT  sim_cam_nodepth_count_LT\n');

function publish_frame_LT_rgbd(pub, msg, pub_d, msg_d)
    persistent last_cs report_tic prev_count pdata last_t drop_dup
    if isempty(last_cs),    last_cs    = uint32(0);                end
    if isempty(last_t),     last_t     = NaN;                      end
    if isempty(drop_dup),   drop_dup   = strcmp(getenv('RGBD_DROP_DUP_STAMPS'),'1'); end
    if isempty(report_tic), report_tic = tic;                      end
    if isempty(prev_count), prev_count = int32(0);                 end
    if isempty(pdata),      pdata      = zeros(3,640*480,'uint8'); end

    try
        frame = evalin('base', 'latest_frame');
        valid = ~isempty(frame) && isa(frame,'uint8') && ndims(frame) == 3 && ...
                size(frame,1) == 480 && size(frame,2) == 640 && size(frame,3) == 3;

        try
            depth = evalin('base', 'latest_depth_left');
        catch
            depth = [];
        end
        valid_d = ~isempty(depth) && isnumeric(depth) && ismatrix(depth) && ...
                  size(depth,1) == 480 && size(depth,2) == 640;

        if ~valid
            d = evalin('base','sim_cam_drop_count_LT');
            assignin('base','sim_cam_drop_count_LT', d + 1);
        elseif ~valid_d
            % Colour without depth is useless to RGB-D odometry: drop the pair.
            d = evalin('base','sim_cam_drop_count_LT');
            assignin('base','sim_cam_drop_count_LT', d + 1);
            n = evalin('base','sim_cam_nodepth_count_LT');
            assignin('base','sim_cam_nodepth_count_LT', n + 1);
        else
            % Same sparse duplicate guard as the mono script (left colour only).
            cs = sum(uint32(frame(1:200:end)));
            if cs == last_cs
                d = evalin('base','sim_cam_drop_count_LT');
                assignin('base','sim_cam_drop_count_LT', d + 1);
            else
                last_cs = cs;

                if strcmp(msg.encoding, 'mono8')
                    % Gray (BT.601 luma, as rgb2gray) row-major, 307200 B.
                    g = rgb2gray(frame).';
                    msg.data = g(:);
                else
                    % Colour: identical packing to sim_camera_publisher_timer_LT.m
                    pdata(1,:) = reshape(frame(:,:,3).', 1, []);
                    pdata(2,:) = reshape(frame(:,:,2).', 1, []);
                    pdata(3,:) = reshape(frame(:,:,1).', 1, []);
                    msg.data = pdata(:);
                end

                % Depth: metres (Z-depth, verified HANDOFF A2b) -> uint16 mm.
                % Sky is a finite 1000.0, so the invalid test is by value,
                % not isfinite. >65.535 m cannot be represented -> 0 (invalid).
                dm  = double(depth);
                bad = ~isfinite(dm) | dm <= 0 | dm > 65.535;
                dm(bad) = 0;
                u  = uint16(round(dm * 1000)).';           % transpose -> row-major
                msg_d.data = typecast(u(:), 'uint8');      % little-endian, 614400 B

                % ONE clock read for BOTH messages: the pair is simultaneous by
                % construction, as in the stereo path of the mono script.
                t_sim = NaN;
                try
                    t_sim = double(evalin('base','sim_heartbeat'));
                    sec   = int32(floor(t_sim));
                    nsec  = uint32((t_sim - double(sec)) * 1e9);
                    msg.header.stamp.sec       = sec;
                    msg.header.stamp.nanosec   = nsec;
                    msg_d.header.stamp.sec     = sec;
                    msg_d.header.stamp.nanosec = nsec;
                catch
                end

                % Repeated-stamp guard (HANDOFF A10): new image content but the same
                % sim_heartbeat as the previous send. Always counted; the pair is
                % only DROPPED when env RGBD_DROP_DUP_STAMPS=1 (default: sent, as mono/stereo do).
                is_dup = ~isnan(t_sim) && isequal(t_sim, last_t);
                if is_dup
                    k = evalin('base','sim_cam_dupstamp_count_LT');
                    assignin('base','sim_cam_dupstamp_count_LT', k + 1);
                end
                last_t = t_sim;
                if is_dup && drop_dup
                    return
                end

                send(pub, msg);
                c = evalin('base','sim_cam_pub_count_LT');
                assignin('base','sim_cam_pub_count_LT', c + 1);

                send(pub_d, msg_d);
                cdep = evalin('base','sim_cam_pub_count_depth_LT');
                assignin('base','sim_cam_pub_count_depth_LT', cdep + 1);
            end
        end

        elapsed = toc(report_tic);
        if elapsed >= 3.0
            c = evalin('base','sim_cam_pub_count_LT');
            if c < prev_count, prev_count = int32(0); end
            rate = double(c - prev_count) / elapsed;
            cdep = evalin('base','sim_cam_pub_count_depth_LT');
            nd = evalin('base','sim_cam_nodepth_count_LT');
            tail = sprintf('  C=%d D=%d nodepth=%d', c, cdep, nd);
            if cdep ~= c, tail = [tail '  <-- DEPTH OUT OF STEP']; end
            fprintf('[cam-LT] published: %d  (+%d in %.1fs = %.1f Hz)%s\n', ...
                    c, c - prev_count, elapsed, rate, tail);
            prev_count = c;
            report_tic = tic;
        end

    catch err
        fprintf('[cam-LT-err] %s\n', err.message);
    end
end
