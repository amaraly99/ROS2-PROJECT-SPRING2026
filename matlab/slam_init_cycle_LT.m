% slam_init_cycle_LT.m  — MANUAL SLAM-init flight (PURE MOTION, NO ROS)
% =========================================================================
% Standard MATLAB UAV control only. This script does NOT touch ROS 2 — no
% ros2subscriber, no ros2publisher, nothing. It commands the drone DIRECTLY by
% writing the sim_cmdvel setpoint in the base workspace (which the Simulink
% block read_cmdvel_live_interp already reads every step). That's it.
%
% WHY NO ROS: creating ROS 2 subscribers inside this MATLAB session crashes the
% fragile ros2 middleware backend ("Connection to process with Exchange ... was
% lost"). So SLAM verification (did it initialise, what's the scale) is done on
% the PI side from /slam/tracking_state + /slam/pose vs ground truth — not here.
% This script's ONLY job is to fly the parallax cycle and return home.
%
% Plan A (live): SLAM sidecar + camera are already up and watching. Run this to
% fly the warmup so SLAM initialises, then it returns the drone to the start.
%
% PRECONDITIONS:
%   1. run hil_ros_init_LT        2. run hil_closed_loop.slx (Stop Time inf, RUN)
%   3. run sim_camera_publisher_timer_LT   4. Pi: ./run_stack_hil.sh ... --hold-fsm
%   5. run slam_init_cycle_LT (THIS)       6. Pi: ./run_stack_hil.sh ... --resume-fsm
%
% RETURNS: assigns `init_result` into the base workspace with the MOTION facts:
%   .passes, .gt_travel_m, .home_dist_m, .moved (logical)
% (SLAM init/scale come from the Pi, not from here.)

%% ------------------------- CYCLE DEFINITION -----------------------------
% Edit every bit here. Each leg = one body-frame velocity held for a fixed time.
% Axis: "x" fwd(+) / "y" LEFT(+) / "z" UP(+). One PASS = the whole list once.
% Keep legs MIRRORED (a +v and a -v of equal duration) so each pass nets ~0.
LEGS = { ...
%    axis   speed(m/s)   dur(s)    comment
    "y",    1.0,         1.0;   % strafe right — lateral parallax
    "y",   -1.0,         1.0;   % strafe left  — mirror back
    "z",    0.5,         0.8;   % climb        — vertical parallax
    "z",   -0.5,         0.8;   % descend      — mirror back
};

NUM_PASSES   = 3;       % how many times to fly the whole leg list (no SLAM feedback
                        % here, so it's fixed — 3 passes ~= 11s, plenty for init).
HOLD_RATE_HZ = 20;      % how often we re-assert the setpoint
SETTLE_SEC   = 1.0;     % zero-hold before finishing

% --- return-to-home after the passes (closed-loop, world frame; home live) ---
RETURN_HOME  = true;
HOME_TOL_M   = 0.05;    % return until within this distance of the captured home
HOME_KP      = 0.8;     % P gain on world-frame position error
HOME_VMAX    = 1.0;     % clip on corrective speed (m/s)
HOME_MAX_SEC = 20;      % bound on the return phase

% =========================================================================
%                     (below here is machinery — NO ROS)
% =========================================================================
assert(evalin('base','exist(''sim_cmdvel'',''var'')') == 1, ...
    'sim_cmdvel missing — run hil_ros_init_LT first.');

dt   = 1.0 / HOLD_RATE_HZ;
Pg   = zeros(0,3);   % GT xyz samples (world) — for the travel report only
t_wall0 = tic;

send_cmd([0;0;0;0;0]);   % clean zero before we start

% Pre-flight guard: is the sim actually RUNNING? Query Simulink directly — do NOT
% infer liveness from position change: we just commanded ZERO velocity above, so
% the drone SHOULD sit still; a "did it move" check would always false-fail.
MODEL_NAME = 'hil_closed_loop';
try
    simstat = get_param(MODEL_NAME, 'SimulationStatus');
catch
    error('Could not query %s — is the model loaded/open in Simulink?', MODEL_NAME);
end
if ~strcmp(simstat, 'running')
    error(['%s SimulationStatus=''%s'' (not running). Start it: ' ...
           'set_param(''%s'',''SimulationCommand'',''start''), then re-run.'], ...
           MODEL_NAME, simstat, MODEL_NAME);
end
g0 = evalin('base','sim_pose');
home = g0(1:3);   % capture the start point DYNAMICALLY — nothing hardcoded
fprintf('\n=== SLAM init cycle (pure motion): home [%.2f %.2f %.2f], %d passes ===\n', ...
    home(1), home(2), home(3), NUM_PASSES);

% ---- fly the passes ----
for pass = 1:NUM_PASSES
    for i = 1:size(LEGS,1)
        ax = LEGS{i,1}; spd = LEGS{i,2}; dur = LEGS{i,3};
        v = zeros(5,1);
        switch ax
            case "x", v(1) = spd;
            case "y", v(2) = spd;
            case "z", v(3) = spd;
            otherwise, error('Unknown leg axis "%s" (use x|y|z)', ax);
        end
        tleg = tic;
        while toc(tleg) < dur
            send_cmd(v);
            Pg = sample_gt(Pg);
            pause(dt);
        end
    end
    fprintf('  pass %d/%d done\n', pass, NUM_PASSES);
end

% ---- return to the captured start point (closed-loop, world frame) ----
% MATLAB's integrators are world-frame (read_cmdvel_live_interp: vx->x_integrator,
% vy->y_integrator, no R(yaw)), so we command the world-frame position error
% directly. Recomputed every tick => self-corrects overshoot.
if RETURN_HOME
    fprintf('  returning to home (tol %.2fm)...\n', HOME_TOL_M);
    tret = tic;
    while toc(tret) < HOME_MAX_SEC
        pos = evalin('base','sim_pose'); pos = pos(1:3);
        err = home(:) - pos(:);
        if norm(err) <= HOME_TOL_M, break; end
        v = max(min(HOME_KP * err, HOME_VMAX), -HOME_VMAX);
        send_cmd([v(1); v(2); v(3); 0; 0]);
        Pg = sample_gt(Pg);
        pause(dt);
    end
end

% ---- settle ----
tset = tic;
while toc(tset) < SETTLE_SEC
    send_cmd([0;0;0;0;0]);
    pause(dt);
end
send_cmd([0;0;0;0;0]);

%% ------------------------- REPORT (motion only) ------------------------
posf = evalin('base','sim_pose'); home_dist = norm(home(:) - posf(1:3));
gt_travel = 0;
if size(Pg,1) >= 2, gt_travel = sum(vecnorm(diff(Pg),2,2)); end
moved = gt_travel > 0.10;

init_result = struct('passes',NUM_PASSES, 'gt_travel_m',gt_travel, ...
    'home_dist_m',home_dist, 'moved',logical(moved));
assignin('base','init_result',init_result);

fprintf('\n---------------------------------------------------------------\n');
fprintf(' CYCLE DONE (motion only — verify SLAM on the Pi)\n');
fprintf('   passes flown            : %d\n', NUM_PASSES);
fprintf('   GT travel during cycle  : %.2f m   (moved: %d)\n', gt_travel, moved);
fprintf('   returned to home within : %.3f m   (start [%.2f %.2f %.2f])\n', ...
    home_dist, home(1), home(2), home(3));
if ~moved
    fprintf('   >> Drone barely moved — is the SIM running? Fix that, re-run.\n');
else
    fprintf('   >> Motion OK. Ask the Pi side to confirm SLAM init + scale,\n');
    fprintf('      then start the FSM: run_stack_hil.sh ... --resume-fsm\n');
end
fprintf('---------------------------------------------------------------\n\n');

% =========================================================================
%                          local functions (NO ROS)
% =========================================================================
function send_cmd(v5)
    % Command the UAV directly: write the velocity setpoint + bump the version
    % counter read_cmdvel_live_interp watches. Pure base-workspace, no ROS.
    v5 = max(min(v5(:), [2;2;1;1;1]), [-2;-2;-1;-1;-1]);
    assignin('base','sim_cmdvel', v5);
    ver = evalin('base','sim_cmdvel_ver');
    assignin('base','sim_cmdvel_ver', int32(ver) + int32(1));
end

function Pg = sample_gt(Pg)
    gt = evalin('base','sim_pose');            % [x;y;z;pitch;yaw] world GT
    Pg(end+1,:) = [gt(1) gt(2) gt(3)];
end
