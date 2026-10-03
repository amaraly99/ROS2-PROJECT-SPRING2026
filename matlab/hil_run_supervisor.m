% hil_run_supervisor.m — lets the Pi start/stop the Simulink HIL simulation
% over a plain TCP socket, so automated experiment sweeps can reset the drone
% to its initial position between configuration runs.
%
% Uses Java sockets (java.net.ServerSocket) — NO Instrument Control Toolbox.
%
% TWO CRITICAL DESIGN RULES (learned the hard way):
%
%  (1) NEVER BLOCK INSIDE THE TIMER CALLBACK.
%      The poll timer can fire re-entrantly from INSIDE the Simulink 3D engine's
%      own step (sim3d.io/Subscriber/receive -> Simulation3DEngine/stepImpl).
%      If the callback then blocks waiting for the model to stop, the sim step it
%      is nested inside can never finish, so the model can never stop -> deadlock,
%      and the sim dies. Therefore the restart is a NON-BLOCKING state machine:
%      we request 'stop', return immediately, and issue 'start' on a LATER poll
%      once the model actually reports 'stopped'.
%
%  (2) The poll uses a 10 ms accept timeout so the main thread is never held long
%      enough to starve the 20 Hz camera publisher timer.
%
% HOW THE DRONE RESETS: a Simulink 'start' re-applies every initial condition
% (drone pose, integrators, scene state) — same as pressing Stop+Run.
%
% v3 (PROPOSED 2026-09-23, NOT YET APPLIED) — the initial-condition fault.
%   Observed: under 'full' orchestration the Pi issues 'start', then waits for
%   state=live, then boots the ROS stack, then opens the bag. The model is
%   therefore RUNNING AND UNSUPERVISED for roughly 8-20 s before the controller
%   exists. In 6 of 10 pilot runs on 2026-09-23 the drone had left its initial
%   condition by the time the first pose was recorded: yaw 306-402 deg against
%   the yaw_integrator IC of 2*pi (= 360 deg), and position 10-20 m downrange of
%   the x/y/z ICs (-15, 10, 10). Every one of those runs then flew a heading that
%   put the target outside the camera's 14.93 deg half-FOV, and none of them
%   tracked the sign. The four runs that started at exactly 2*pi all did.
%
%   NOT the cause (checked, 2026-09-23): a latched /cmd_vel. The last /cmd_vel of
%   every pilot run has angular.z = 0.000 exactly, so the next run cannot be
%   inheriting a yaw rate through base-workspace sim_cmdvel. The model's
%   LoadInitialState and SaveFinalState are both 'off', so state is not carried
%   through that path either.
%
%   Remaining explanation, and what v3 fixes: the drone moves during the
%   unsupervised pre-roll, driven by whatever the booting controller publishes
%   before anything is recording. v3 removes the pre-roll instead of trying to
%   characterise it:
%     (a) 'start_hold' + 'resume' let the Pi hold the model at its ICs until the
%         stack is live, so every run begins integrating from the same state.
%     (b) 'start' now zeroes base-workspace sim_cmdvel and bumps sim_cmdvel_ver,
%         so a stale command cannot drive the drone even if one is ever present.
%         Cheap and defensive; it is not the observed cause.
%     (c) 'status' now reports the drone's pose, so the Pi can check the initial
%         condition over the existing protocol instead of reading a bag.
%
% SETUP (run AFTER the normal HIL session is already streaming ~20 Hz):
%   1. clear all + your 3x setenv(...)
%   2. run hil_ros_init_LT             (wait for "=== LT Init complete ===")
%   3. open + RUN hil_closed_loop.slx  (wait for latest_frame in workspace)
%   4. run sim_camera_publisher_timer_LT  (confirm ~20 Hz on the Pi FIRST)
%   5. run hil_run_supervisor          (returns to prompt; keeps polling)
%
%   NOTE: the ">>" prompt is free after step 5 — it is just buried under the
%   continuous [cam-LT] / [cmdvel] output. Press Enter to see it. To stop:
%       stop_hil_supervisor
%
% PROTOCOL (one line per command, TCP port 55556; reply is one line):
%   "start"         -> stop model if running, then restart it (fresh initial conditions)   reply: ok
%   "start_hold"    -> like "start", but PAUSE the model as soon as it is running, so the
%                      drone sits at its initial conditions until "resume". v3, see below.  reply: ok
%   "resume"        -> release a "start_hold" (no-op if the model is not paused)            reply: ok
%   "stop"          -> stop the model                                                       reply: ok
%   "status"        -> "status sim=<SimulationStatus> t=<sim time s> state=<supervisor state>
%                       pub=<on|off> timers=<n> hz=<PUBLISH_HZ> fresh=<n> repeat=<n> drop=<n>
%                       yaw=<deg> x=<m> y=<m> z=<m>"   (v3: pose added for the Pi's IC guard)
%                      supervisor state: restarting|verifying|live|frozen|stopped|idle
%   "pub_start [hz]"-> (re)start the camera publisher, callback state reset; hz optional
%                      (default: last rate used, else 20)                                   reply: ok | error <msg>
%   "pub_stop"      -> stop + delete the camera publisher timer                             reply: ok | error <msg>
%   anything else   -> reply: error
% A refused connection => supervisor not running => the Pi prompts the operator.

MODEL_NAME = 'hil_closed_loop';
PORT = 55556;

if strcmp(MODEL_NAME, 'CHANGE_ME_TO_YOUR_MODEL_NAME')
    error('hil_run_supervisor: edit MODEL_NAME at the top of this script first.');
end

load_system(MODEL_NAME);

% Clean up any previous supervisor (timer + socket) so re-running is safe.
stop_hil_supervisor();

serverSock = java.net.ServerSocket(PORT);
serverSock.setSoTimeout(10);   % 10 ms — never starves the camera timer
assignin('base', 'hil_supervisor_sock', serverSock);

supTimer = timer( ...
    'Name',          'hil_supervisor_timer', ...
    'Period',        0.2, ...
    'ExecutionMode', 'fixedRate', ...
    'BusyMode',      'drop', ...
    'TimerFcn',      @(~,~) supervisor_poll(serverSock, MODEL_NAME), ...
    'ErrorFcn',      @(~,e) fprintf('[supervisor-err] %s\n', e.message));
assignin('base', 'hil_supervisor_timer_obj', supTimer);
start(supTimer);

fprintf('[supervisor] model "%s" loaded\n', MODEL_NAME);
fprintf('[supervisor] polling port %d every 0.2 s (non-blocking state machine)\n', PORT);
fprintf('[supervisor] command line is FREE (press Enter to see >>). Stop: stop_hil_supervisor\n');

% ─────────────────────────────────────────────────────────────────────────
function supervisor_poll(serverSock, model)
    % NON-BLOCKING. Each call does at most one quick action and returns, so it
    % can safely fire even while nested inside a Simulink step.
    SETTLE_POLLS = 25;    % 25 x 0.2 s = ~5 s between 'stopped' and 'start'
    VERIFY_POLLS = 150;   % 150 x 0.2 s = ~30 s allowed for new frames after 'start'
                          % Was 100 (~20 s). The first 'start' after a MATLAB
                          % relaunch delivered its first new frame ~17 s in, i.e.
                          % inside the old budget but close to it; later starts
                          % take ~8 s. The Pi waits up to 45 s, so 30 s here still
                          % fails before the Pi's own timeout.
    persistent restartPending settlePolls verifyPolls lastCs tStart holdPending
    if isempty(restartPending), restartPending = false; end
    if isempty(settlePolls),    settlePolls    = 0;     end
    if isempty(verifyPolls),    verifyPolls    = -1;    end   % -1 = not verifying
    if isempty(lastCs),         lastCs         = [];    end
    if isempty(holdPending),    holdPending    = false; end   % v3: pause once live

    % (A) Advance a pending restart without ever waiting.
    %     Once the model reports 'stopped' we let the Simulation 3D engine
    %     settle for SETTLE_POLLS polls (~5 s) before issuing 'start'. Firing
    %     'start' too soon can race the sim3d viewer teardown, which makes the
    %     3D window close without reopening. Non-blocking: just count polls.
    if restartPending
        if strcmp(get_param(model, 'SimulationStatus'), 'stopped')
            settlePolls = settlePolls + 1;
            if settlePolls >= SETTLE_POLLS
                % v3: clear any stale velocity command before the drone can
                % integrate it. Defensive -- see the v3 note at the top.
                zero_cmdvel();
                set_param(model, 'SimulationCommand', 'start');
                restartPending = false;
                settlePolls = 0;
                verifyPolls = 0;                 % start watching latest_frame
                lastCs = frame_checksum();
                tStart = tic;
                assignin('base', 'hil_supervisor_state', 'verifying');
                fprintf('[supervisor] start issued — waiting for new frames (up to %.0f s)\n', VERIFY_POLLS * 0.2);
            end
        end
        % else: model is still stopping; we'll check again on the next poll.
    end

    % (A2) After a restart, the model only counts as running once latest_frame
    %      changes. One checksum per poll, never waits.
    if verifyPolls >= 0
        verifyPolls = verifyPolls + 1;
        cs = frame_checksum();
        if ~isempty(cs) && ~isempty(lastCs) && cs ~= lastCs && ...
                strcmp(get_param(model, 'SimulationStatus'), 'running')
            fprintf('[supervisor] LIVE: new frames %.1f s after start — drone at initial pose\n', toc(tStart));
            % v3: if this was a 'start_hold', freeze the model here, while the
            % drone is still at its initial condition, and wait for 'resume'.
            % This is the whole point of the fix: no unsupervised pre-roll.
            if holdPending
                try
                    set_param(model, 'SimulationCommand', 'pause');
                    assignin('base', 'hil_supervisor_state', 'held');
                    fprintf('[supervisor] HELD at initial condition — waiting for "resume"\n');
                catch he
                    fprintf(2, '[supervisor] pause failed (%s); continuing unheld\n', he.message);
                    assignin('base', 'hil_supervisor_state', 'live');
                end
                holdPending = false;
            else
                assignin('base', 'hil_supervisor_state', 'live');
            end
            verifyPolls = -1;
        elseif verifyPolls >= VERIFY_POLLS
            fprintf(2, ['[supervisor] WARNING: NO NEW FRAMES %.0f s after start (status=%s). ' ...
                        'The 3D window has probably closed; this run will be FROZEN.\n'], ...
                    toc(tStart), get_param(model, 'SimulationStatus'));
            assignin('base', 'hil_supervisor_state', 'frozen');
            verifyPolls = -1;
        end
        if ~isempty(cs), lastCs = cs; end
    end

    % (B) Check the socket for a new command (blocks at most 10 ms).
    try
        conn = serverSock.accept();
    catch
        return;   % SocketTimeout — nothing waiting this tick
    end
    try
        reader = java.io.BufferedReader( ...
                    java.io.InputStreamReader(conn.getInputStream()));
        writer = java.io.PrintWriter(conn.getOutputStream(), true);
        line = reader.readLine();
        if ~isempty(line)
            cmd = strtrim(char(line));
            fprintf('[supervisor] received: "%s"\n', cmd);
            switch cmd
                case {'start', 'start_hold'}
                    % Request stop now (async, returns immediately). The actual
                    % 'start' is issued in a LATER poll once the model reports
                    % 'stopped' — see block (A). Never block here.
                    if ~strcmp(get_param(model, 'SimulationStatus'), 'stopped')
                        set_param(model, 'SimulationCommand', 'stop');
                    end
                    restartPending = true;
                    settlePolls = 0;
                    verifyPolls = -1;                    % cancel any earlier verification
                    % v3: 'start_hold' pauses the model the moment it goes live,
                    % so the drone waits at its ICs until the Pi says 'resume'.
                    holdPending = strcmp(cmd, 'start_hold');
                    zero_cmdvel();                       % v3, defensive
                    assignin('base', 'hil_supervisor_state', 'restarting');
                    writer.println('ok');
                case 'resume'
                    % v3: release a hold. Harmless if the model is not paused.
                    try
                        if strcmp(get_param(model, 'SimulationStatus'), 'paused')
                            set_param(model, 'SimulationCommand', 'continue');
                        end
                        holdPending = false;
                        assignin('base', 'hil_supervisor_state', 'live');
                        writer.println('ok');
                    catch re_
                        writer.println(['error ' re_.message]);
                    end
                case 'status'
                    writer.println(status_line(model));
                case 'pub_stop'
                    try
                        hil_pub_control('stop');
                        writer.println('ok');
                    catch pe
                        writer.println(['error ' pe.message]);
                    end
                case 'stop'
                    if ~strcmp(get_param(model, 'SimulationStatus'), 'stopped')
                        set_param(model, 'SimulationCommand', 'stop');
                    end
                    restartPending = false;
                    settlePolls = 0;
                    verifyPolls = -1;
                    holdPending = false;                 % v3
                    assignin('base', 'hil_supervisor_state', 'stopped');
                    writer.println('ok');
                otherwise
                    if strncmp(cmd, 'pub_start', 9)
                        % "pub_start" or "pub_start <hz>"
                        try
                            hz = str2double(strtrim(cmd(10:end)));
                            if isnan(hz)
                                hz = 20;
                                if evalin('base', "exist('sim_cam_pub_hz','var')")
                                    hz = evalin('base', 'sim_cam_pub_hz');
                                end
                            end
                            hil_pub_control('start', hz);
                            writer.println('ok');
                        catch pe
                            writer.println(['error ' pe.message]);
                        end
                    else
                        fprintf('[supervisor] unknown command "%s" (ignored)\n', cmd);
                        writer.println('error');
                    end
            end
        end
    catch innerErr
        fprintf('[supervisor] connection error: %s\n', innerErr.message);
    end
    try, conn.close(); catch, end
end

function line = status_line(model)
    % One-line state for the "status" command. Reads only; never blocks.
    st = 'idle';
    try
        if evalin('base', "exist('hil_supervisor_state','var')")
            st = evalin('base', 'hil_supervisor_state');
        end
    catch
    end
    try, simst = get_param(model, 'SimulationStatus'); catch, simst = '?'; end
    try, t = get_param(model, 'SimulationTime'); catch, t = NaN; end
    % v3: drone pose, so the Pi's IC guard can read the initial condition over
    % the existing protocol instead of opening a bag. sim_pose is written every
    % model step by read_cmdvel_live_interp as [x; y; z; pitch; yaw] (rad).
    pose = '';
    try
        if evalin('base', "exist('sim_pose','var')")
            sp = evalin('base', 'sim_pose');
            if numel(sp) >= 5
                pose = sprintf(' yaw=%.2f x=%.2f y=%.2f z=%.2f', ...
                               rad2deg(double(sp(5))), double(sp(1)), double(sp(2)), double(sp(3)));
            end
        end
    catch
    end
    try
        p = hil_pub_control('status');
        pub = 'off'; if p.running, pub = 'on'; end
        line = sprintf('status sim=%s t=%.2f state=%s pub=%s timers=%d hz=%g fresh=%d repeat=%d drop=%d%s', ...
                       simst, t, st, pub, p.ntimers, p.hz, p.fresh, p.repeat, p.drop, pose);
    catch
        line = sprintf('status sim=%s t=%.2f state=%s pub=? timers=? hz=? fresh=? repeat=? drop=?%s', simst, t, st, pose);
    end
end

function zero_cmdvel()
    % v3. Clear any latched /cmd_vel so a stale command cannot drive the drone
    % during a restart. read_cmdvel_live_interp re-reads sim_cmdvel only when
    % sim_cmdvel_ver changes, so the version counter must be bumped too.
    % Defensive only: the pilot data shows every run's last angular.z is exactly
    % 0.000, so a latched yaw rate is NOT the observed cause of the IC fault.
    try
        assignin('base', 'sim_cmdvel', zeros(5, 1));
        v = 0;
        if evalin('base', "exist('sim_cmdvel_ver','var')")
            v = double(evalin('base', 'sim_cmdvel_ver'));
        end
        assignin('base', 'sim_cmdvel_ver', int32(v + 1));
    catch
    end
end

function cs = frame_checksum()
    % Same sparse checksum as sim_camera_publisher_timer_LT; [] if no valid frame.
    cs = [];
    try
        f = evalin('base', 'latest_frame');
        if ~isempty(f) && isa(f, 'uint8') && ndims(f) == 3
            cs = sum(uint32(f(1:200:end)));
        end
    catch
    end
end
