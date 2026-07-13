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
% PROTOCOL (one line per command, TCP port 55556):
%   "start" -> stop model if running, then restart it (fresh initial conditions)
%   "stop"  -> stop the model
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
    persistent restartPending settlePolls
    if isempty(restartPending), restartPending = false; end
    if isempty(settlePolls),    settlePolls    = 0;     end

    % (A) Advance a pending restart without ever waiting.
    %     Once the model reports 'stopped' we let the Simulation 3D engine
    %     settle for a few polls (~1 s) before issuing 'start'. Firing 'start'
    %     the instant it stops can race the sim3d viewer teardown, which makes
    %     the 3D window close without reopening. Non-blocking: just count polls.
    if restartPending
        if strcmp(get_param(model, 'SimulationStatus'), 'stopped')
            settlePolls = settlePolls + 1;
            if settlePolls >= 10    % ~2 s at 0.2 s/poll — let the sim3d engine fully release
                set_param(model, 'SimulationCommand', 'start');
                restartPending = false;
                settlePolls = 0;
                fprintf('[supervisor] simulation restarted — drone at initial pose\n');
            end
        end
        % else: model is still stopping; we'll check again on the next poll.
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
                case 'start'
                    % Request stop now (async, returns immediately). The actual
                    % 'start' is issued in a LATER poll once the model reports
                    % 'stopped' — see block (A). Never block here.
                    if ~strcmp(get_param(model, 'SimulationStatus'), 'stopped')
                        set_param(model, 'SimulationCommand', 'stop');
                    end
                    restartPending = true;
                    settlePolls = 0;
                    writer.println('ok');
                case 'stop'
                    if ~strcmp(get_param(model, 'SimulationStatus'), 'stopped')
                        set_param(model, 'SimulationCommand', 'stop');
                    end
                    restartPending = false;
                    settlePolls = 0;
                    writer.println('ok');
                otherwise
                    fprintf('[supervisor] unknown command "%s" (ignored)\n', cmd);
                    writer.println('error');
            end
        end
    catch innerErr
        fprintf('[supervisor] connection error: %s\n', innerErr.message);
    end
    try, conn.close(); catch, end
end
