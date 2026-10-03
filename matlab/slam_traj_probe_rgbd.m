% slam_traj_probe_rgbd.m: GENERATED RGB-D copy of slam_traj_probe.m (2026-09-27).
% Only change: MODEL_NAME = 'hil_closed_loop_baseline_RGBD' (the status check).
% If slam_traj_probe.m changes, regenerate this copy; do not hand-edit.

% slam_traj_probe.m  — SLAM-friendly scripted trajectory probe (PURE MOTION, NO ROS)
% =========================================================================
% Flies one of three scripted trajectories (set TRAJ_MODE below) — lateral
% parallax converging to target-y, descending to target-z, fixed yaw — then a
% closed-loop approach to a standoff in front of the stop sign. Purpose: answer
% "given trajectory T under an ideal detector, how sane are the SLAM poses?" —
% SLAM is RECORDED on the Pi and evaluated separately; this script ONLY drives
% the motion.
%
% Standard MATLAB only. NO ros2subscriber (creating one crashes the ROS backend).
% Commands the drone by writing sim_cmdvel (world-frame: vx=+x fwd, vy=+y left,
% vz=+z up — matches read_cmdvel_live_interp integrators). Reads GT `sim_pose`
% and `sim_target_pose` from the base workspace (already there, no subscription).
%
% Council fixes baked in:
%   - Legs gated on GT DISPLACEMENT (not wall-clock) -> pacing-independent path.
%   - Per-run FREEZE WATCHDOG: sim_pose unchanged too long -> abort + mark INVALID.
%   - Final approach keeps a NON-DECAYING lateral wiggle so parallax doesn't
%     collapse at the sign.
%   - "reached" = logged GT distance-to-target (no FSM here; distinct metric).
%
% PRECONDITIONS: hil_ros_init_LT + running sim + camera publisher; Pi on --hold-fsm
% (SLAM+camera+bag up, NO reactive FSM). Then: run slam_traj_probe.
%
% RETURNS base var `init_result`: .invalid .end_pos .dist_to_target_m .maxtime_hit

%% ------------------------- TRAJECTORY (approved) ------------------------
% A/B: fly ALL modes through this SAME script+eval path (the fair comparison).
%   "weave"   = aggressive zig-zag (+-1.8 m/s). DEAD: scale exploded to 1e17.
%   "forward" = degenerate pure-forward baseline (lateral only at the final
%               approach, i.e. forward-dominant like the reactive controller).
%   "gentle"  = mild lateral sinusoid in SPACE. The third point between those
%               two known extremes — see GENTLE_* block below.
TRAJ_MODE = "sixdof";

% SLAM-friendly weave (approved; matches traj_design.m). {[vx vy vz] m/s, dur_s}.
LEGS_WEAVE = { ...
    [1.4,  1.8, -0.12], 4.0 ;   [1.4, -1.8, -0.12], 3.0 ;
    [1.4,  1.6, -0.12], 3.2 ;   [1.4, -1.4, -0.12], 2.6 ;
    [1.4,  1.2, -0.12], 2.4 ;   [1.4, -0.9, -0.12], 2.0 ;
    [1.4,  0.6, -0.10], 1.8 ;   [1.4, -0.3, -0.10], 1.5 };
% Degenerate baseline: pure forward + matched descent (same fwd speed / net drop,
% NO lateral). The closed-loop approach then aligns y + reaches the standoff, so
% this mimics the forward-dominant real approach. Isolates: does sustained lateral
% parallax (weave) keep mono SLAM sane vs forward-dominant (baseline)?
LEGS_FORWARD = { [1.4,  0.0, -0.08], 22.5 };

% --- "gentle": lateral sinusoid parameterised by FORWARD DISTANCE, not time ---
%       vy(dx) = GENTLE_A * sin(2*pi*dx/GENTLE_LAMBDA) + GENTLE_BIAS
% Driving it off GT forward displacement (not wall clock) keeps the same
% pacing-independence the displacement-gated legs were built for: if the sim runs
% slow, the PATH SHAPE is unchanged, only the time to fly it stretches.
%
% Why this shape. The failed weave conflated two suspects, and this removes both:
%   1. peak per-frame translation  0.114 -> 0.078 m (camera is 20 Hz, dt=50 ms);
%      "forward" is 0.072, so this is only +8% over the baseline, not +58%.
%   2. the square-wave reversal — vy jumped +-3.6 m/s BETWEEN TWO FRAMES at every
%      one of the 8 leg boundaries, flipping the whole optical-flow field. Here
%      the worst frame-to-frame vy step is 0.25 m/s, 14x smaller, and that worst
%      case is the cruise->approach handoff, not the weave itself.
% GENTLE_LAMBDA * GENTLE_CYCLES = 31.5 m = the forward baseline's exact open-loop
% distance, and whole cycles land the cruise on sin=0, so the handoff into the
% approach carries no vy step either. Forward distance, descent and duration
% therefore match "forward" EXACTLY — vy is the only variable that differs.
% Amplitude 0.5 -> 1.2. What the sinusoid has to buy is LATERAL EXTENT, and the
% path's extent is what was measured to be degenerate: PCA on the forward run's
% GT gives 9.599 x 1.132 x 0.004 m, i.e. a straight line. A straight line cannot
% constrain mono scale (no bearing diversity) AND cannot expose a scale error to
% Sim(3) alignment (a mis-scaled line is still the same line) — scoring the
% cruise with its own scale gives 0.057 m, forcing the full-flight scale on the
% same poses gives 0.295 m. Extent is the variable; amplitude is the knob.
%   excursion = (A/vx)*(LAMBDA/2*pi):  0.5 -> +-0.60 m,  1.2 -> +-1.43 m
% Why this is not the dead weave: that was a SQUARE wave stepping vy by 3.6 m/s
% between two frames at each of 8 leg boundaries, which flipped the whole optical
% flow field. Here the worst frame-to-frame vy step is A*2*pi*vx/LAMBDA*dt =
% 0.050 m/s, 72x smaller, and peak per-frame translation is 0.098 m against the
% dead weave's 0.114 m. Amplitude was never the thing that killed the weave.
% A=1.2 EXPLODED the map (run 20260802_125121): stable 0.38 m through the whole
% 31 m cruise, then |slam_pos| 7.0 -> 329 -> 2.9e6 at the cruise->terminal
% handoff, scale 4.58 -> 0.000. Back to 0.5, the amplitude that flew clean in
% 20260801_220136, to isolate amplitude from the OTHER thing that changed since
% that run: pacing is now 1.0x real-time instead of 1/3, so per-frame translation
% is ~4x larger at any given amplitude. If 0.5 also explodes now, amplitude is
% not the cause and per-frame motion is.
GENTLE_A      = 0.5;    % lateral amplitude (m/s) — peak, not RMS
GENTLE_LAMBDA = 10.5;   % spatial wavelength, in FORWARD metres
GENTLE_CYCLES = 3;      % whole cycles => cruise ends at vy = bias, no step
GENTLE_BIAS   = 0.17;   % mean lateral drift: +3.8 m over the cruise -> y ~ 23.7
GENTLE_VX     = 1.4;    % same forward speed as both other modes
GENTLE_VZ     = -0.08;  % same descent as "forward": 5.0 -> 3.2 = target z

% --- YAW SWEEP: arm B of the bearing-diversity A/B ------------------------
% Velocity commands are WORLD-frame (read_cmdvel_live_interp applies NO R(yaw)),
% so yaw rotates the CAMERA without altering the flown path by a millimetre.
% Arm A (0 deg) and arm B (12 deg) therefore fly a geometrically IDENTICAL
% trajectory and differ only in where the camera looks — which isolates bearing
% diversity, the thing EuRoC has and a fixed-yaw run does not, from everything
% else. Fixed yaw keeps every bearing ray to a given feature near-parallel for
% the whole run, which is the worst case for triangulation conditioning.
%
% Closed-loop on the yaw ANGLE, not an open-loop rate: the sim paces at roughly
% 1/3 real time, so a wz derived from the COMMANDED vx would advance yaw ~3x too
% fast per metre travelled. Tracking the angle is immune to pacing.
% Amplitude stays well inside the 60 deg HFOV (+-30 deg) so the sign stays framed.
% Whole GENTLE_CYCLES land the cruise on sin=0, so yaw is already back to 0 at
% the handoff into the approach — no step there either.
YAW_AMP_DEG = 0;      % 0 = arm A (baseline).  12 = arm B (yaw sweep).
YAW_KP      = 1.5;    % rad/s per rad of yaw error
YAW_WZ_MAX  = 0.5;    % rad/s clip
% Orchestrator hook: hil_trial.ps1 sets YAW_AMP_OVERRIDE in the base workspace to
% pick the arm per trial, so an unattended A/B never has to edit this file
% mid-sweep (which would make the two arms differ by an edit, not just a value).
if exist('YAW_AMP_OVERRIDE','var'), YAW_AMP_DEG = YAW_AMP_OVERRIDE; end

switch TRAJ_MODE
    case "forward", LEGS = LEGS_FORWARD;
    case "gentle",  LEGS = {};              % cruise loop below replaces the legs
    case "sixdof",  LEGS = {};              % ditto — 6-DOF helix cruise loop
    otherwise,      LEGS = LEGS_WEAVE;
end
STANDOFF = 3.0;                 % stop this far in front (-x) of the sign

HOLD_RATE_HZ       = 20;        % setpoint re-assert + sample rate
% Safety cap as a multiple of the leg's NOMINAL duration. Was 3.0, which the sim
% actually hit: it paces at roughly 1/3 real time (measured 0.44 m/s against a
% commanded 1.4), so a nominal 22.5 s leg needs ~68 s and 3.0x left no margin --
% the cruise terminated on the CLOCK at 31.43/31.50 m instead of on the
% displacement gate, which is the very thing the gate exists to prevent.
LEG_MAXTIME_FACTOR = 5.0;
FREEZE_SEC         = 0.7;       % sim_pose unchanged this long => sim frozen -> INVALID
% The watchdog is meant to catch a sim that DIES MID-RUN. Armed from t=0 it also
% catches ordinary startup latency: right after a SimulationCommand restart,
% sim_pose can stall for a second or two while MATLAB is still busy, and a run
% then aborts at t+3.6s having never moved. Disarm it until the sim has had a
% fair chance to start integrating.
STARTUP_GRACE_SEC  = 6.0;
APPROACH_KP        = 0.8;       % P gain toward standoff
APPROACH_VMAX      = 1.2;       % corrective speed clip (m/s)
% Persistent lateral parallax through the approach. This is the last 3 m, and it
% is where the run was dying: ATE tracks drift at ~0.004 m/m through the cruise
% (0.055 m at 19 m, 0.100 m at 27 m) and then jumps to 0.257 m over the final
% 3.2 m — far faster than drift explains — before collapsing entirely at the stop.
%
% Two bugs did that, both fixed here:
%  1. forward mode used to zero this ("baseline stays degenerate"), so the last
%     3 m were a dead-straight decelerating line with NO lateral motion at all.
%     That A/B is over; keeping it degenerate now only breaks the endgame.
%  2. the oscillation was FAR too fast to produce usable baseline. sin(k*0.3) at
%     dt=0.05 is 6 rad/s — a 1.0 s period — and the resulting lateral EXCURSION
%     is only amp/w = 0.4/6 = 0.067 m. Seven centimetres of sway at 3 m range is
%     not parallax. What matters is the excursion, not the velocity amplitude.
% Excursion = APPROACH_WIGGLE / APPROACH_WIGGLE_W, so 0.5/1.0 = 0.5 m each way,
% 1.0 m peak-to-peak, over a 6.3 s period.
APPROACH_WIGGLE    = 0.5;       % lateral velocity amplitude (m/s)
APPROACH_WIGGLE_W  = 1.0;       % rad/s -> 6.3 s period, 0.5 m excursion
APPROACH_TOL       = 0.15;      % arrive within this of standoff (x and z only)
APPROACH_MAXSEC    = 25;        % approach-phase time bound

% --- PHASE 0: dedicated SLAM initialisation, BEFORE the mission ----------
% Measured on run 20260801_234612: the drone started moving at t=4.85 s and the
% first non-zero /slam/pose arrived at t=11.56 s. SLAM was therefore bootstrapping
% 2.24 m INTO the mission, while flying straight at a sign 33 m away. Forward
% motion puts the epipole at the image centre, which is the degenerate case for
% essential-matrix decomposition — the logged init translation was
% [-0.088 -0.001 0.234], i.e. almost pure forward — and every later pose inherits
% that badly-conditioned birth. Give SLAM a proper map FIRST.
%
% Sideways translation is the opposite case: the baseline is perpendicular to the
% viewing axis, which maximises parallax per metre travelled and conditions the
% essential matrix well. Parallax for a feature at range Z is f*B/Z, so at f=554
% px and Z~35.6 m the sign alone gives 554*3/35.6 = 47 px against OV2SLAM's
% finit_parallax gate of 10 px — and the ground plane inside the FOV is nearer
% than the sign, so it clears the gate sooner still.
%
% This is NOT slam_init_cycle_LT.m's mirrored wiggle. That one went out AND back,
% so on the return leg the baseline against the init keyframe collapsed back
% toward zero; it peaked at 0.25 px, ~40x short of the gate. This leg is
% ONE-DIRECTIONAL and the drone stays displaced.
%
% Deliberately NO rotation in this phase: rotation without translation is itself
% degenerate for the essential matrix, and mixing it in while the baseline is
% still short only worsens the conditioning. The 6-DOF content belongs in the
% mission, once a map exists to track against.
INIT_ENABLE   = true;
INIT_LATERAL  = 3.0;    % m of sideways (-y) displacement — the init baseline
INIT_VY       = -0.8;   % m/s sideways. -y moves AWAY from the sign's y (23.7),
                        % which also leaves the mission more lateral to cover.
INIT_VZ       = 0.2;    % m/s up: makes the init baseline 2-D (y AND z) rather
                        % than a single line, better conditioned still.
INIT_SETTLE   = 2.0;    % s held still after the gate, to let the mapper finish
                        % its first keyframes before the mission perturbs things.
INIT_MAXSEC   = 20;     % time bound on the init leg

% --- "sixdof": the mission, exciting every DOF the sim exposes ------------
% sim_cmdvel is [vx vy vz wy wz] (read_cmdvel_live_interp) — 3 translations plus
% pitch rate and yaw rate. There is no roll channel in the model, so this is all
% 5 available DOF, and it fixes both defects measured in the forward runs:
%   1. GT path extent was 9.599 x 1.132 x 0.004 m — planar to within 4 mm. The
%      vertical sinusoid is 90 deg out of phase with the lateral one, so the path
%      becomes a true 3-D helix and the third axis grows to ~0.34 m RMS.
%   2. Yaw was pinned at 0 for the whole flight, so every feature was observed
%      from a near-constant bearing for 30 m — the worst case for triangulation
%      conditioning, and the thing EuRoC has that these runs did not.
% Amplitudes are sized so peak per-frame translation is 0.075 m, essentially the
% forward baseline's 0.070 m: the two runs that exploded were not killed by
% amplitude (0.5 and 1.2 both exploded) so there is no reason to pay for it here.
% Sweeps stay inside the FOV: yaw 8 deg against a 30 deg half-HFOV, pitch 3 deg
% against a 23.4 deg half-VFOV, so the sign never leaves frame.
SIX_VX        = 1.2;    % m/s forward
SIX_AY        = 0.5;    % m/s lateral amplitude  -> 0.70 m excursion
SIX_AZ        = 0.35;   % m/s vertical amplitude -> 0.49 m excursion, 90 deg out
SIX_LAMBDA    = 10.5;   % m, spatial wavelength (gated on GT forward distance)
SIX_CYCLES    = 3;      % whole cycles => sinusoids land on 0 at the handoff
SIX_YAW_DEG   = 8.0;    % yaw sweep amplitude
SIX_PITCH_DEG = 3.0;    % pitch sweep amplitude, 90 deg out of phase with yaw
SIX_PITCH_KP  = 1.5;    % rad/s per rad of pitch error
SIX_WY_MAX    = 0.4;    % rad/s clip on pitch rate

% Orchestrator hooks, same pattern as YAW_AMP_OVERRIDE: the close-range arm needs
% a shorter cruise (the drone is transited most of the way before the stack comes
% up) and an init leg that goes TOWARD the sign's y rather than away from it, so
% the compressed cruise is not forced to carry a steep lateral bias.
% PROBE_PHASE lets the orchestrator run the init cycle and the mission as two
% separate calls, so it can check BETWEEN them whether SLAM actually built a map.
% If a backend has not initialised by the end of the init cycle there is no point
% flying the mission: it would be scored over a shorter path than every other arm
% and quietly flatter itself. Measured: OV2SLAM-fast needs 10.75 m against a 3.0 m
% init leg, so it enters the mission with no map at all.
%   'init'    -> Phase 0 only, then return
%   'mission' -> skip Phase 0, fly cruise + approach
%   ''        -> everything (default, for manual use)
PROBE_PHASE = '';
if exist('PROBE_PHASE_OVERRIDE','var'), PROBE_PHASE = PROBE_PHASE_OVERRIDE; end
if exist('SIX_VX_OVERRIDE','var'),     SIX_VX     = SIX_VX_OVERRIDE;     end
if exist('SIX_LAMBDA_OVERRIDE','var'), SIX_LAMBDA = SIX_LAMBDA_OVERRIDE; end
if exist('SIX_CYCLES_OVERRIDE','var'), SIX_CYCLES = SIX_CYCLES_OVERRIDE; end
if exist('INIT_VY_OVERRIDE','var'),    INIT_VY    = INIT_VY_OVERRIDE;    end

% =========================================================================
%                          (drive — NO ROS)
% =========================================================================
assert(evalin('base','exist(''sim_cmdvel'',''var'')') == 1, ...
    'sim_cmdvel missing — run hil_ros_init_LT first.');
dt = 1/HOLD_RATE_HZ;

send_cmd([0;0;0;0;0]);
% preflight: is the sim actually RUNNING? Query Simulink directly — do NOT infer
% liveness from position change, because we just commanded ZERO velocity above,
% so the drone SHOULD sit still; a "did it move" check would always false-fail.
MODEL_NAME = 'hil_closed_loop_baseline_RGBD';   % RGB-D copy
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
% Capture the start point DYNAMICALLY. NOTE: this used to read a bare `g0`, which
% this script never defines — it was a leftover from slam_init_cycle_LT.m, and it
% only ever "worked" because running that script first left g0 in the base
% workspace. The procedure now says to SKIP the init cycle, so on a fresh session
% that was a hard error at this line, before a single command was sent.
g0   = evalin('base','sim_pose');
home = g0(1:3);
% Target pose is NOT a base-workspace var — hil_ros_init_LT.m only ever send()s
% it once to /sim/target_pose (transientlocal, for the Pi's late subscribers), it
% never assignin('base',...)'s it. Hardcoded here to match that literal exactly
% (hil_ros_init_LT.m line 81: [35.5; 23.7; 3.2; pi]). If that line ever changes,
% update this too — deliberately NOT touching hil_ros_init_LT.m per your request.
target = [35.5; 23.7; 3.2];
goal   = target(:) - [STANDOFF; 0; 0];
fprintf('\n=== traj probe: start [%.1f %.1f %.1f]  target [%.1f %.1f %.1f]  standoff %.1fm ===\n', ...
    home(1),home(2),home(3), target(1),target(2),target(3), STANDOFF);

invalid = false;  maxtime_hit = 0;
lastp = evalin('base','sim_pose');  t_change = tic;
t_run = tic;      % run clock, for STARTUP_GRACE_SEC on the freeze watchdog

% ---- PHASE 0: initialise SLAM before the mission starts ----
% Gated on GT lateral displacement, not the clock, so the baseline actually flown
% is INIT_LATERAL metres regardless of how the sim is pacing.
if INIT_ENABLE && ~strcmp(PROBE_PHASE, 'mission')
    fprintf('  slam init: %.1f m sideways (vy=%.1f, vz=%.1f), no rotation...\n', ...
        INIT_LATERAL, INIT_VY, INIT_VZ);
    y0 = lastp(2);  ti = tic;
    while true
        p = evalin('base','sim_pose');
        if abs(p(2) - y0) >= INIT_LATERAL, break; end
        if ~isequal(p, lastp), t_change = tic; end
        lastp = p;
        if toc(t_run) > STARTUP_GRACE_SEC && toc(t_change) > FREEZE_SEC
            invalid = true; break;
        end
        if toc(ti) > INIT_MAXSEC
            maxtime_hit = maxtime_hit + 1; break;
        end
        send_cmd([0; INIT_VY; INIT_VZ; 0; 0]);
        pause(dt);
    end
    send_cmd([0;0;0;0;0]);
    pause(INIT_SETTLE);          % let the mapper close out its first keyframes
    p = evalin('base','sim_pose');  lastp = p;  t_change = tic;
    fprintf('\n');
    fprintf('=========================================================\n');
    fprintf(' INIT CYCLE COMPLETE\n');
    fprintf('   lateral baseline flown : %.2f m (target %.2f m)\n', abs(p(2)-y0), INIT_LATERAL);
    fprintf('   settled at             : [%.2f %.2f %.2f]\n', p(1), p(2), p(3));
    fprintf('   >> orchestrator now checks whether SLAM built a map.\n');
    fprintf('      If it did NOT, this trial is ABORTED before the mission.\n');
    fprintf('=========================================================\n\n');
    init_result_phase0 = struct('lateral_m', abs(p(2)-y0), 'pos', p(1:3).');
    assignin('base','init_result_phase0', init_result_phase0);
end

% Phase 0 only -- hand control back so the orchestrator can verify SLAM init.
if strcmp(PROBE_PHASE, 'init')
    send_cmd([0;0;0;0;0]);
    assignin('base','init_result', struct('invalid',logical(invalid), ...
        'phase','init', 'end_pos',lastp(1:3).', 'dist_to_target_m',NaN, ...
        'standoff_m',STANDOFF, 'maxtime_hit',maxtime_hit, ...
        'traj_mode',TRAJ_MODE, 'yaw_amp_deg',YAW_AMP_DEG));
    return;
end

% ---- 6-DOF mission cruise: 3-D helix + yaw/pitch sweeps, gated on GT dx ----
if TRAJ_MODE == "sixdof" && ~invalid
    dx_want = SIX_LAMBDA * SIX_CYCLES;
    x0 = lastp(1);
    % Bias terms are solved from where the init leg actually LEFT the drone, not
    % from the nominal spawn, so the mission still lands on the sign's y/z after
    % the sideways init displaced it.
    t_cruise  = dx_want / SIX_VX;
    bias_y    = (goal(2) - lastp(2)) / t_cruise;
    bias_z    = (goal(3) - lastp(3)) / t_cruise;
    tcap = tic;  tmax = LEG_MAXTIME_FACTOR * t_cruise;
    fprintf(['  6dof cruise: %.1f m fwd = %d x %.1f m, vy=%.2f*sin%+.2f, ' ...
             'vz=%.2f*cos%+.2f, yaw %.0f deg, pitch %.0f deg\n'], ...
        dx_want, SIX_CYCLES, SIX_LAMBDA, SIX_AY, bias_y, SIX_AZ, bias_z, ...
        SIX_YAW_DEG, SIX_PITCH_DEG);
    while true
        p = evalin('base','sim_pose');
        if ~isequal(p, lastp), t_change = tic; end
        lastp = p;
        if toc(t_run) > STARTUP_GRACE_SEC && toc(t_change) > FREEZE_SEC
            invalid = true; break;
        end
        dx = p(1) - x0;
        if dx >= dx_want, break; end
        if toc(tcap) > tmax, maxtime_hit = maxtime_hit + 1; break; end
        ph = 2*pi*dx/SIX_LAMBDA;
        % sin for lateral, cos for vertical: 90 deg apart, so the path is a helix
        % rather than a wiggle inside a plane. Same split for yaw vs pitch.
        send_cmd([SIX_VX; ...
                  SIX_AY*sin(ph) + bias_y; ...
                  SIX_AZ*cos(ph) + bias_z; ...
                  yaw_ctl(deg2rad(SIX_PITCH_DEG)*cos(ph), p(4), SIX_PITCH_KP, SIX_WY_MAX); ...
                  yaw_ctl(deg2rad(SIX_YAW_DEG)*sin(ph),  p(5), YAW_KP, YAW_WZ_MAX)]);
        pause(dt);
    end
    if ~invalid
        fprintf('  cruise done (fwd %.2f/%.2f m, pos [%.2f %.2f %.2f]%s)\n', ...
            dx, dx_want, p(1),p(2),p(3), tern(maxtime_hit>0,'  [MAXTIME]',''));
    end
end

% ---- open-loop cruise: gentle spatial sinusoid, gated on GT forward distance ----
if TRAJ_MODE == "gentle"
    dx_want = GENTLE_LAMBDA * GENTLE_CYCLES;
    x0      = lastp(1);
    tcap    = tic;  tmax = LEG_MAXTIME_FACTOR * (dx_want / GENTLE_VX);
    fprintf(['  gentle cruise: %.1f m fwd = %d x %.1f m cycles, ' ...
             'vy = %.2f*sin + %.2f, yaw amp %.1f deg [ARM %s]\n'], ...
        dx_want, GENTLE_CYCLES, GENTLE_LAMBDA, GENTLE_A, GENTLE_BIAS, ...
        YAW_AMP_DEG, tern(YAW_AMP_DEG > 0, 'B (yaw sweep)', 'A (baseline)'));
    while true
        p = evalin('base','sim_pose');
        if ~isequal(p, lastp), t_change = tic; end   % pose moved -> reset watchdog
        lastp = p;
        if toc(t_run) > STARTUP_GRACE_SEC && toc(t_change) > FREEZE_SEC
            invalid = true; break;
        end
        dx = p(1) - x0;
        if dx >= dx_want, break; end                 % forward distance reached
        if toc(tcap) > tmax, maxtime_hit = maxtime_hit + 1; break; end
        vy      = GENTLE_A*sin(2*pi*dx/GENTLE_LAMBDA) + GENTLE_BIAS;
        yaw_des = deg2rad(YAW_AMP_DEG)*sin(2*pi*dx/GENTLE_LAMBDA);
        send_cmd([GENTLE_VX; vy; GENTLE_VZ; 0; ...
                  yaw_ctl(yaw_des, p(5), YAW_KP, YAW_WZ_MAX)]);
        pause(dt);
    end
    if ~invalid
        fprintf('  cruise done (fwd %.2f/%.2f m, pos [%.2f %.2f %.2f]%s)\n', ...
            dx, dx_want, p(1),p(2),p(3), tern(maxtime_hit>0,'  [MAXTIME]',''));
    end
end

% ---- open-loop legs, gated on GT displacement ----
for i = 1:size(LEGS,1)
    v    = LEGS{i,1}(:);  dur = LEGS{i,2};
    want = norm(v(1:3)) * dur;              % intended metres for this leg
    ls   = evalin('base','sim_pose');  ls = ls(1:3);
    tcap = tic;  hit = false;
    while true
        send_cmd([v(1);v(2);v(3); 0; 0]);
        pause(dt);
        p = evalin('base','sim_pose');
        if ~isequal(p, lastp), t_change = tic; end   % pose moved -> reset watchdog
        lastp = p;
        if toc(t_run) > STARTUP_GRACE_SEC && toc(t_change) > FREEZE_SEC
            invalid = true; break;
        end
        if norm(p(1:3) - ls) >= want, break; end      % displacement reached
        if toc(tcap) > LEG_MAXTIME_FACTOR*dur, hit = true; break; end  % safety
    end
    if invalid, break; end
    if hit, maxtime_hit = maxtime_hit + 1; end
    fprintf('  leg %d/%d done (moved %.2f/%.2f m%s)\n', i, size(LEGS,1), ...
        norm(p(1:3)-ls), want, tern(hit,'  [MAXTIME]',''));
end

% ---- closed-loop approach to standoff (persistent lateral wiggle) ----
if ~invalid
    fprintf('  approach to standoff...\n');
    ta = tic;  k = 0;
    while toc(ta) < APPROACH_MAXSEC
        p = evalin('base','sim_pose');
        err = goal - p(1:3);
        % Arrival is judged on x and z ONLY. y now carries a deliberate 0.5 m
        % wiggle, so a norm(err) test could never fall inside a 0.15 m tolerance
        % and the approach would always burn APPROACH_MAXSEC instead of exiting.
        % Standoff distance is along x anyway, which is what the metric cares about.
        if abs(err(1)) < APPROACH_TOL && abs(err(3)) < APPROACH_TOL, break; end
        vv = max(min(APPROACH_KP*err, APPROACH_VMAX), -APPROACH_VMAX);
        vv(2) = vv(2) + APPROACH_WIGGLE*sin(k*dt*APPROACH_WIGGLE_W);  % parallax
        % hold the camera on +x through the approach (yaw_des = 0). The cruise
        % already ends at yaw 0, so this is continuous, and it keeps the sign
        % centred exactly as in arm A while closing on the standoff.
        % Level the camera as well as holding yaw. The 6-DOF cruise ends on
        % cos(6*pi)=+1, i.e. pitch parked at SIX_PITCH_DEG rather than 0, and
        % sending wy=0 here would only FREEZE that tilt for the whole approach.
        send_cmd([vv(1);vv(2);vv(3); ...
                  yaw_ctl(0, p(4), SIX_PITCH_KP, SIX_WY_MAX); ...
                  yaw_ctl(0, p(5), YAW_KP, YAW_WZ_MAX)]);
        pause(dt);  k = k + 1;
        if ~isequal(p, lastp), t_change = tic; end
        lastp = p;
        if toc(t_run) > STARTUP_GRACE_SEC && toc(t_change) > FREEZE_SEC
            invalid = true; break;
        end
    end
end

% ---- terminal parallax hold at the standoff ----
% Stopping dead kills the mono scale gauge within ~2 s (measured: Umeyama scale
% 7.88 -> 0.16 across the stop). Holding the standoff WITH the lateral wiggle
% still running keeps parallax — and therefore scale — observable through the
% end of the recording, so the collapse never enters the evaluation window.
% Mission-wise this is a hover-in-place at the arrived standoff, not extra travel.
% 0 = off. The hold existed to keep mono scale observable through the end of the
% recording, but the motion-gated eval window already excludes the parked tail
% entirely, and /slam/pose messages are published live per frame so a later gauge
% collapse cannot retroactively rewrite poses already in the bag. Measured, the
% hold HURT: its 25 s of poses all sit at one location, so they dominate the
% Umeyama fit by count (E1b no-hold 0.239 m vs E3 hold 0.474 m).
TERMINAL_HOLD_SEC = 0;
if ~invalid
    fprintf('  terminal parallax hold (%.0f s)...\n', TERMINAL_HOLD_SEC);
    th = tic;
    while toc(th) < TERMINAL_HOLD_SEC
        p = evalin('base','sim_pose');
        err = goal - p(1:3);
        vv = max(min(APPROACH_KP*err, APPROACH_VMAX), -APPROACH_VMAX);
        vv(2) = vv(2) + APPROACH_WIGGLE*sin(k*dt*APPROACH_WIGGLE_W);
        send_cmd([vv(1);vv(2);vv(3); 0; ...
                  yaw_ctl(0, p(5), YAW_KP, YAW_WZ_MAX)]);
        pause(dt);  k = k + 1;
        if ~isequal(p, lastp), t_change = tic; end
        lastp = p;
        if toc(t_run) > STARTUP_GRACE_SEC && toc(t_change) > FREEZE_SEC
            invalid = true; break;
        end
    end
end
send_cmd([0;0;0;0;0]);

% ---- report ----
p   = evalin('base','sim_pose');
dtt = norm(p(1:3) - target);
init_result = struct('invalid',logical(invalid), 'end_pos',p(1:3).', ...
    'dist_to_target_m',dtt, 'standoff_m',STANDOFF, 'maxtime_hit',maxtime_hit, ...
    'traj_mode',TRAJ_MODE, 'yaw_amp_deg',YAW_AMP_DEG);   % provenance for the sweep
assignin('base','init_result',init_result);

fprintf('\n-------------------------------------------------\n');
if invalid
    fprintf(' TRAJ PROBE: INVALID — sim_pose froze mid-run. DISCARD this run.\n');
else
    fprintf(' TRAJ PROBE DONE\n');
    fprintf('   end pos       : [%.2f %.2f %.2f]\n', p(1),p(2),p(3));
    fprintf('   dist to sign  : %.2f m   (standoff %.1f m)\n', dtt, STANDOFF);
    if maxtime_hit>0
        fprintf('   WARNING       : %d leg(s) hit the max-time cap (drone slower than\n', maxtime_hit);
        fprintf('                   commanded) -> path shape is off-nominal, treat with care.\n');
    end
    fprintf('   >> now evaluate SLAM sanity from the Pi bag.\n');
end
fprintf('-------------------------------------------------\n\n');

% =========================================================================
%                          local functions (NO ROS)
% =========================================================================
function send_cmd(v5)
    % Direct world-frame velocity setpoint into the sim. Pure base workspace.
    % Must stay >= the clamp in read_cmdvel_live_interp.m, which is the one the
    % sim actually enforces; a lower value here would silently cap the probe.
    v5 = max(min(v5(:), [12;12;2;1;1]), [-12;-12;-2;-1;-1]);
    assignin('base','sim_cmdvel', v5);
    ver = evalin('base','sim_cmdvel_ver');
    assignin('base','sim_cmdvel_ver', int32(ver) + int32(1));
end

function s = tern(c, a, b)
    if c, s = a; else, s = b; end
end

function wz = yaw_ctl(yaw_des, yaw_now, kp, wmax)
    % P-control on the yaw ANGLE, with the error WRAPPED to [-pi,pi].
    % The yaw integrator is unbounded and is routinely left at a multiple of
    % 2*pi by a previous run (observed: sim_pose(5) = 6.283 = 2*pi, which is
    % the SAME heading as 0). Raw subtraction would read that as a 360 deg
    % error and command a full reverse spin to "correct" a correct heading —
    % and it would do so in arm A too, where yaw is supposed to stay fixed.
    e  = mod(yaw_des - yaw_now + pi, 2*pi) - pi;
    wz = max(min(kp*e, wmax), -wmax);
end
