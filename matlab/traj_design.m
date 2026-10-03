% traj_design.m  — PREVIEW / DESIGN the SLAM-friendly trajectory (no drone, no ROS)
% =========================================================================
% Run this in MATLAB (`run traj_design`) to SEE the path before flying it.
% It integrates the same LEGS list the real probe (slam_traj_probe.m) will fly,
% plus the closed-loop final approach, and plots top-down (x-y) and side (x-z).
% Tweak the LEGS below and re-run to reshape the path. Nothing here commands the
% drone — it's pure math + a plot.
%
% Frame: world (matches read_cmdvel_live_interp integrators). Fixed yaw (camera +x).
%   vx = +forward(east)   vy = +left(north)   vz = +up
% =========================================================================

start_pos = [0; 20; 5];        % drone start (world)
target    = [35.5; 23.7; 3.2]; % stop sign (world)
STANDOFF  = 3.0;               % stop this far in front (-x) of the sign

%% ---------------- THE TRAJECTORY (edit here) ----------------------------
% Keep these values in sync with slam_traj_probe.m — this file previews the
% shape, that one flies it.
TRAJ_MODE = "gentle";        % "gentle" | "weave" | "forward"

% Each leg: { [vx vy vz] (m/s) , duration_s }.  Forward zig-zag: weave y for
% continuous lateral parallax while progressing +x, y-amplitude decaying so it
% converges onto the target's lateral line; gentle descent to target height.
LEGS_WEAVE = { ...
    [1.4,  1.8, -0.12], 4.0 ;   [1.4, -1.8, -0.12], 3.0 ;
    [1.4,  1.6, -0.12], 3.2 ;   [1.4, -1.4, -0.12], 2.6 ;
    [1.4,  1.2, -0.12], 2.4 ;   [1.4, -0.9, -0.12], 2.0 ;
    [1.4,  0.6, -0.10], 1.8 ;   [1.4, -0.3, -0.10], 1.5 };
LEGS_FORWARD = { [1.4,  0.0, -0.08], 22.5 };

% "gentle": vy = A*sin(2*pi*dx/LAMBDA) + BIAS, driven by forward distance dx.
GENTLE_A      = 0.5;    GENTLE_LAMBDA = 10.5;   GENTLE_CYCLES = 3;
GENTLE_BIAS   = 0.17;   GENTLE_VX     = 1.4;    GENTLE_VZ     = -0.08;

switch TRAJ_MODE
    case "forward", LEGS = LEGS_FORWARD;
    case "gentle",  LEGS = {};
    otherwise,      LEGS = LEGS_WEAVE;
end

% final closed-loop approach to the standoff point
APPROACH_KP     = 0.8;    % P gain toward standoff
APPROACH_VMAX   = 1.2;    % clip (m/s)
APPROACH_WIGGLE = 0.4;    % persistent lateral wiggle amplitude (keeps parallax at the end)
APPROACH_TOL    = 0.10;   % arrive within this of standoff

% =========================================================================
%                        (integrate + plot)
% =========================================================================
dt = 0.05;
pos = start_pos;
P = pos.';                       % path samples (rows = [x y z])

% --- open-loop cruise: gentle spatial sinusoid, gated on forward distance ---
if TRAJ_MODE == "gentle"
    dx_want = GENTLE_LAMBDA * GENTLE_CYCLES;
    x0 = pos(1);
    while pos(1) - x0 < dx_want
        vy  = GENTLE_A*sin(2*pi*(pos(1)-x0)/GENTLE_LAMBDA) + GENTLE_BIAS;
        pos = pos + [GENTLE_VX; vy; GENTLE_VZ]*dt;
        P(end+1,:) = pos.'; %#ok<SAGROW>
    end
end

% --- open-loop legs ---
for i = 1:size(LEGS,1)
    v = LEGS{i,1}(:); dur = LEGS{i,2};
    for k = 1:round(dur/dt)
        pos = pos + v*dt;
        P(end+1,:) = pos.'; %#ok<SAGROW>
    end
end

% --- closed-loop approach to standoff (with lateral wiggle) ---
goal = target - [STANDOFF; 0; 0];   % 3 m in front of the sign
for k = 1:800
    err = goal - pos;
    if norm(err) < APPROACH_TOL, break; end
    v = max(min(APPROACH_KP*err, APPROACH_VMAX), -APPROACH_VMAX);
    v(2) = v(2) + APPROACH_WIGGLE*sin(k*0.3);   % non-decaying lateral parallax
    pos = pos + v*dt;
    P(end+1,:) = pos.'; %#ok<SAGROW>
end

% --- stats ---
path_len = sum(vecnorm(diff(P),2,2));
fprintf('\npath length : %.1f m   (straight-line would be %.1f m)\n', ...
    path_len, norm(goal-start_pos));
fprintf('end pos     : [%.2f %.2f %.2f]   dist to sign: %.2f m\n', ...
    P(end,1),P(end,2),P(end,3), norm(P(end,:).'-target));
fprintf('y span      : [%.1f %.1f]  (target y = %.1f)\n', min(P(:,2)),max(P(:,2)),target(2));
fprintf('z           : %.1f -> %.1f  (target z = %.1f)\n\n', P(1,3),P(end,3),target(3));

% --- plot ---
figure('Name','SLAM-friendly trajectory','Color','w','Position',[100 100 1200 480]);
subplot(1,2,1); hold on; grid on; axis equal;
plot(P(:,1),P(:,2),'-','LineWidth',1.8,'Color',[0.12 0.47 0.71]);
plot(start_pos(1),start_pos(2),'go','MarkerSize',11,'MarkerFaceColor','g');
plot(target(1),target(2),'rp','MarkerSize',20,'MarkerFaceColor','r');
plot(goal(1),goal(2),'kx','MarkerSize',12,'LineWidth',3);
yline(target(2),':r');
xlabel('x  (forward, m)'); ylabel('y  (lateral, m)');
title('TOP-DOWN: lateral parallax, converging to target-y');
legend('path','start','stop sign','standoff','Location','best');

subplot(1,2,2); hold on; grid on;
plot(P(:,1),P(:,3),'-','LineWidth',1.8,'Color',[0.12 0.47 0.71]);
plot(start_pos(1),start_pos(3),'go','MarkerSize',11,'MarkerFaceColor','g');
plot(target(1),target(3),'rp','MarkerSize',20,'MarkerFaceColor','r');
plot(goal(1),goal(3),'kx','MarkerSize',12,'LineWidth',3);
yline(target(3),':r');
xlabel('x  (forward, m)'); ylabel('z  (altitude, m)');
title('SIDE: gentle descent to target height');
sgtitle('Proposed SLAM-friendly trajectory  (fixed yaw, world-frame legs)','FontWeight','bold');
