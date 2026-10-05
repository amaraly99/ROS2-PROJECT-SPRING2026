function vel = read_cmdvel_live_interp(t, pitch_angle, x_pos, y_pos, z_pos, yaw_angle)
% Reads live /cmd_vel values from MATLAB base workspace.
%
% Inputs (via Mux, u(1)..u(6)):
%   u(1) = clock          (s, sim time; used only for the held start below)
%   u(2) = pitch_angle    (rad, from pitch_integrator, +ve = nose down)
%   u(3) = x_pos          (m, from x_integrator)
%   u(4) = y_pos          (m, from y_integrator)
%   u(5) = z_pos          (m, from z_integrator post-saturation = actual altitude)
%   u(6) = yaw_angle      (rad, from yaw_integrator)
%
% Output (5 elements):
%   vel(1) = vx  <- linear.x   -> x_integrator   (forward)
%   vel(2) = vy  <- linear.y   -> y_integrator   (lateral, IC=15)
%   vel(3) = vz  <- linear.z   -> z_integrator   (altitude, IC=10)
%   vel(4) = wy  <- angular.y  -> pitch_integrator
%   vel(5) = wz  <- angular.z  -> yaw_integrator

% Mark evalin/assignin as extrinsic so the MATLAB Code Generator treats
% them as external calls rather than attempting (and failing) to compile
% them.  Without this declaration the entire function degrades to
% interpreted mode, erasing any JIT benefit for the arithmetic below.
coder.extrinsic('assignin', 'evalin', 'find_system', 'get_param', 'set_param');

% --- Write pose to base workspace (2 calls; unavoidable) ----------------
% These feed the 20 Hz pitch/pose/heartbeat ROS state publisher.
assignin('base', 'sim_pitch_angle', double(pitch_angle));
assignin('base', 'sim_pose', [double(x_pos); double(y_pos); double(z_pos); ...
                               double(pitch_angle); double(yaw_angle)]);

% --- Read velocity with persistent version-cache (was 10 evalin calls) --
% cmdvel_callback writes sim_cmdvel=[vx;vy;vz;wy;wz] and bumps
% sim_cmdvel_ver whenever /cmd_vel arrives.  We only re-read the 5-element
% vector when the version counter changes, so the common case (no new
% command this step) costs exactly ONE evalin instead of ten.
persistent cached_vel cached_ver
if isempty(cached_vel)
    cached_vel = zeros(5, 1);
    cached_ver = int32(-1);
end

ver = int32(evalin('base', 'sim_cmdvel_ver'));  % 1 evalin (scalar)
if ver ~= cached_ver
    cached_vel = double(evalin('base', 'sim_cmdvel'));  % 1 evalin (5-element vector)
    cached_ver = ver;
end

% Safety clamp (vectorized — no per-element branches)
vel = max(min(cached_vel, [2; 2; 1; 1; 1]), [-2; -2; -1; -1; -1]);

% --- Held start (2026-10-05) ---------------------------------------------
% hil_run_supervisor's 'start_hold' sets base-workspace sim_hold_at (s). When
% the sim clock reaches it, clear it and pause the running model, so every hold
% lands on the same step. Inert when sim_hold_at does not exist, which is the
% case for a plain 'start' and for any session without the supervisor.
if evalin('base', 'exist(''sim_hold_at'', ''var'')')
    if t >= evalin('base', 'sim_hold_at') - 1e-9
        evalin('base', 'clear sim_hold_at');
        mdls = find_system('SearchDepth', 0, 'type', 'block_diagram');
        for k = 1:numel(mdls)
            if strcmp(get_param(mdls{k}, 'SimulationStatus'), 'running')
                set_param(mdls{k}, 'SimulationCommand', 'pause');
            end
        end
    end
end
end
