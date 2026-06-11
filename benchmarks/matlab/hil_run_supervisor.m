% hil_run_supervisor.m — lets the Pi start/stop the Simulink HIL simulation
% over a plain TCP socket, so automated experiment sweeps can reset the drone
% to its initial position between configuration runs.
%
% WHY TCP (not ROS2): the supervisor must stay reachable while the Simulink
% model itself is stopped/restarted; a raw socket has no DDS discovery delays
% and no dependency on the model's ROS blocks being up.
%
% HOW THE DRONE RESETS: a Simulink "start" re-applies every initial condition
% in the model (drone pose, integrators, scene state). The supervisor stops
% the model if running, waits, then starts it — equivalent to you pressing
% Stop+Run in the Simulink toolbar.
%
% SETUP (once per experiment session, on this Windows machine):
%   1. Open MATLAB, cd to the folder containing your HIL Simulink model.
%   2. Edit MODEL_NAME below to the model's name (without .slx).
%   3. Run this script. Leave it running; it prints each command it receives.
%   4. On the Pi, benchmarks/run_hil_experiments.sh sends "start"/"stop"
%      automatically (MATLAB_HOST_IP:55556). Ctrl+C here to quit.
%
% PROTOCOL: one line per command over TCP port 55556:
%   "start" -> stop model if running, restart it (fresh initial conditions)
%   "stop"  -> stop the model
% The Pi treats a refused connection as "supervisor not running" and falls
% back to prompting the operator.

MODEL_NAME = 'CHANGE_ME_TO_YOUR_MODEL_NAME';   % e.g. 'hil_drone_sim'
PORT = 55556;

if strcmp(MODEL_NAME, 'CHANGE_ME_TO_YOUR_MODEL_NAME')
    error('hil_run_supervisor: edit MODEL_NAME at the top of this script first.');
end

load_system(MODEL_NAME);
fprintf('[supervisor] model "%s" loaded, listening on port %d...\n', ...
        MODEL_NAME, PORT);

srv = tcpserver('0.0.0.0', PORT, 'Timeout', 10);
configureTerminator(srv, 'LF');
srv.UserData = MODEL_NAME;
configureCallback(srv, 'terminator', @onCommand);

fprintf('[supervisor] ready. Ctrl+C to quit.\n');
% Keep the script alive; callbacks fire from the event loop.
while true
    pause(1);
end

function onCommand(src, ~)
    model = src.UserData;
    cmd = strtrim(readline(src));
    fprintf('[supervisor] received: "%s"\n', cmd);
    try
        status = get_param(model, 'SimulationStatus');
        switch cmd
            case 'start'
                if ~strcmp(status, 'stopped')
                    set_param(model, 'SimulationCommand', 'stop');
                    waitForStop(model);
                end
                set_param(model, 'SimulationCommand', 'start');
                fprintf('[supervisor] simulation (re)started — drone at initial pose\n');
            case 'stop'
                if ~strcmp(status, 'stopped')
                    set_param(model, 'SimulationCommand', 'stop');
                    waitForStop(model);
                end
                fprintf('[supervisor] simulation stopped\n');
            otherwise
                fprintf('[supervisor] unknown command "%s" (ignored)\n', cmd);
        end
        if src.Connected
            writeline(src, 'ok');
        end
    catch err
        fprintf('[supervisor] ERROR: %s\n', err.message);
        if src.Connected
            writeline(src, 'error');
        end
    end
end

function waitForStop(model)
    for i = 1:50   % up to 5 s
        if strcmp(get_param(model, 'SimulationStatus'), 'stopped')
            return;
        end
        pause(0.1);
    end
    warning('model did not reach "stopped" within 5 s');
end
