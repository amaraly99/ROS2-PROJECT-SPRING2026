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
% To STOP:    hil_pub_control('stop')   (stops AND deletes the timer)
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

% Timer creation, callback-state reset and cleanup live in hil_pub_control.m;
% the callback itself is hil_publish_frame.m. Same timer name, rate and behaviour.
hil_pub_control('start', PUBLISH_HZ);
fprintf('sim_cam_pub_timer_LT started at %d Hz (constant-rate) on /sim/camera/image_raw\n', PUBLISH_HZ);
fprintf('  Stop:     hil_pub_control(''stop'')   (or stop(sim_cam_pub_timer_LT))\n');
fprintf('  Counters: sim_cam_pub_count_LT  sim_cam_repeat_count_LT  sim_cam_drop_count_LT\n');
