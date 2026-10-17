% hil_session_start.m — one command that prepares MATLAB for a Pi-driven HIL session.
%
%   >> cd <repo>\matlab
%   >> hil_session_start
%
% Then leave MATLAB alone: benchmarks/ctrl_runner on the Pi does every run through
% the supervisor (TCP port 55556): camera publisher on/off, start_hold, resume, stop.
%
% Steps (the SETUP order of hil_run_supervisor.m):
%   1. environment: Fast DDS (RMW + a resolved copy of config/hil/fastrtps_matlab.xml),
%      ROS_DOMAIN_ID 0, HIL_MODEL (the model the supervisor controls), build cache in tempdir
%   2. hil_ros_init_LT                 (ROS node, publishers, 20 Hz state timer)
%   3. open the shared model, read back + check its start pose, run it in real time,
%      wait for the first camera frame (max 180 s)
%   4. sim_camera_publisher_timer_LT   (camera publisher; the Pi switches it per run)
%   5. hil_run_supervisor              (listens on TCP 55556; the prompt stays free)
% A failing step stops with an error that names the step. The model file is not saved.
% Every variable starts with hs_: this runs in the base workspace next to the HIL scripts.
%
% Approved by Amar Aly, 2026-10-17: model hil_closed_loop_unified (shared 20 m start,
% SIM_PARAMS.md), Fast DDS with the fastrtps_matlab.xml profile.

clear all; clc; %#ok<CLALL>   % hil_ros_init_LT needs a clean workspace

% ── settings ──────────────────────────────────────────────────────────────
hs_model        = 'hil_closed_loop_unified';   % shared start: x 15.12, y 3.84, z 7.5, yaw 2*pi-0.0460
hs_matlab_ip    = '192.168.1.201';             % this Windows PC on the Wi-Fi
hs_pi_ip        = '192.168.1.60';              % the Raspberry Pi on the Wi-Fi
hs_expect_ic    = struct('x_integrator', 15.12, 'y_integrator', 3.84, 'z_integrator', 7.5, ...
                         'yaw_integrator', 2*pi - 0.0460);
hs_frame_wait_s = 180;
% ──────────────────────────────────────────────────────────────────────────

hs_dir = fileparts(mfilename('fullpath'));
cd(hs_dir);

fprintf('[session] 1/5 environment\n');
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');
setenv('ROS_DOMAIN_ID', '0');
setenv('HIL_MODEL', hs_model);
hs_tmpl = fullfile(hs_dir, '..', 'config', 'hil', 'fastrtps_matlab.xml');
if ~isfile(hs_tmpl)
    error('[session] step 1: Fast DDS template not found: %s', hs_tmpl);
end
hs_xml = fileread(hs_tmpl);
hs_xml = strrep(hs_xml, '${MATLAB_HOST_IP}', hs_matlab_ip);
hs_xml = strrep(hs_xml, '${PI_ADDR}', hs_pi_ip);
if contains(hs_xml, '${')
    error('[session] step 1: the Fast DDS profile still has an unfilled ${...} placeholder');
end
hs_profile = fullfile(tempdir, 'fastrtps_matlab_resolved.xml');
hs_fid = fopen(hs_profile, 'w'); fwrite(hs_fid, hs_xml); fclose(hs_fid);
setenv('FASTRTPS_DEFAULT_PROFILES_FILE', hs_profile);
hs_gen_dir = fullfile(tempdir, 'hil_session_simgen');   % the tracked slprj is from another MATLAB release
Simulink.fileGenControl('set', 'CacheFolder', hs_gen_dir, 'CodeGenFolder', hs_gen_dir, 'createDir', true);
fprintf('[session]     RMW=%s  DOMAIN=%s  HIL_MODEL=%s\n', getenv('RMW_IMPLEMENTATION'), getenv('ROS_DOMAIN_ID'), hs_model);
fprintf('[session]     Fast DDS profile %s  (MATLAB %s, Pi %s)\n', hs_profile, hs_matlab_ip, hs_pi_ip);

fprintf('[session] 2/5 hil_ros_init_LT\n');
hil_ros_init_LT;

fprintf('[session] 3/5 open + check + run %s\n', hs_model);
if ~isfile(fullfile(hs_dir, [hs_model '.slx']))
    error('[session] step 3: %s.slx not found in %s', hs_model, hs_dir);
end
open_system(fullfile(hs_dir, [hs_model '.slx']));
for hs_blk = fieldnames(hs_expect_ic)'
    hs_v = get_param([hs_model '/' hs_blk{1}], 'InitialCondition');
    fprintf('[session]     %-15s InitialCondition = %s\n', hs_blk{1}, hs_v);
    if abs(evalin('base', hs_v) - hs_expect_ic.(hs_blk{1})) > 1e-6
        error('[session] step 3: %s starts at %s, expected %.4f (shared start, SIM_PARAMS.md)', ...
              hs_blk{1}, hs_v, hs_expect_ic.(hs_blk{1}));
    end
end
set_param(hs_model, 'EnablePacing', 'on', 'PacingRate', '1');   % real time; the file is not saved
set_param(hs_model, 'SimulationCommand', 'start');
hs_t_frame = tic;
hs_have_frame = false;
while toc(hs_t_frame) < hs_frame_wait_s
    pause(0.5);
    if evalin('base', "exist('latest_frame','var')") && ~isempty(evalin('base', 'latest_frame'))
        hs_have_frame = true;
        break;
    end
end
if ~hs_have_frame
    error('[session] step 3: no camera frame after %d s — check the 3D window', hs_frame_wait_s);
end
fprintf('[session]     first frame after %.1f s\n', toc(hs_t_frame));

fprintf('[session] 4/5 sim_camera_publisher_timer_LT\n');
sim_camera_publisher_timer_LT;

fprintf('[session] 5/5 hil_run_supervisor (model %s)\n', getenv('HIL_MODEL'));
hil_run_supervisor;

fprintf('[session] READY — start the run on the Pi:  benchmarks/ctrl_runner proportional\n');
