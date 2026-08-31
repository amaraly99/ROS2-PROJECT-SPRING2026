function out = check_stereo_geometry(varargin)
%CHECK_STEREO_GEOMETRY  Prove the stereo rig is geometrically correct, no Pi needed.
%
% WHY
%   "The right image looks a bit shifted" is not a result. This turns the whole
%   stereo geometry into numbers you can put in the paper, and it catches every
%   silent failure mode before the Pi ever sees a frame:
%
%     symptom                          cause
%     -------------------------------  --------------------------------------
%     almost no confident matches      the two cameras share a Sensor identifier,
%                                      so Unreal feeds both blocks the SAME image
%     median vertical residual ~= 0    the cameras have different Relative
%                                      rotation; the pair is not rectified
%     median disparity NEGATIVE        left and right are swapped; the right
%                                      camera's Y offset has the wrong sign
%     slope ~= 1 vs predicted          the baseline in Simulink and the baseline
%                                      in the YAML disagree, or fx is wrong
%
%   The last one needs ground-truth depth: enable the RIGHT camera's
%   Ground Truth -> Output depth port and wire it to a write_depth_right block
%   (Step 2d of the stereo walkthrough). Without it you still get the first
%   three checks.
%
% BONUS: it measures fx independently.
%   With depth available,  fx = disparity * z / baseline.  That is a completely
%   separate route to the focal length from check_camera_fov, so the two
%   together settle the 554-vs-1200 question from both directions.
%
% USAGE
%   check_stereo_geometry
%   check_stereo_geometry('Baseline', 0.11, 'Fx', 1200)
%   s = check_stereo_geometry;      % returns the numbers as a struct
%
% See also SET_STEREO, CHECK_CAMERA_FOV, LIVE_CAMERA_VIEW_STEREO_LT.

    p = inputParser;
    addParameter(p, 'Baseline', 0.11);   % metres, must match Simulink AND the YAML
    addParameter(p, 'Fx',       1200);   % CONFIRMED 2026-08-24 (was wrongly 554 everywhere)
    addParameter(p, 'Plot',     true);
    parse(p, varargin{:});
    BASELINE_M = p.Results.Baseline;
    FX_PX      = p.Results.Fx;

    [L, R, D] = grab_pair();

    fprintf('\n=== check_stereo_geometry ===\n');
    fprintf('  baseline assumed  : %.4f m\n', BASELINE_M);
    fprintf('  fx assumed        : %g px\n',  FX_PX);
    if isempty(D)
        fprintf('  ground-truth depth: NOT available (Step 2d not done)\n');
    else
        fprintf('  ground-truth depth: available\n');
    end

    % --- Guard: are the two eyes even different images? ----------------------
    if isequal(L, R)
        fprintf('\n  FAIL: the two eyes are PIXEL-IDENTICAL.\n');
        fprintf(['    Both camera blocks are bound to the same Unreal sensor. Open the\n' ...
                 '    right camera block -> Mounting -> Sensor identifier and set it to 2\n' ...
                 '    (the left one is 1). Then restart the model.\n\n']);
        out = struct('ok', false, 'reason', 'identical_images');
        return;
    end

    % --- Match patches from left into right ----------------------------------
    [du, dv, uu, vv, n] = match_grid(L, R);

    fprintf('\n  confident matches : %d\n', n);
    if n < 15
        fprintf(['\n  FAIL: too few confident matches (%d). Either the scene is almost\n' ...
                 '  featureless from this pose, or the two eyes are near-identical.\n' ...
                 '  Move the drone somewhere with structure and try again.\n\n'], n);
        out = struct('ok', false, 'reason', 'too_few_matches', 'n', n);
        return;
    end

    med_dv   = median(dv);
    med_disp = median(du);

    fprintf('\n  RECTIFICATION  median vertical residual : %+6.2f px   (want ~0)\n', med_dv);
    fprintf('  SIGN           median disparity         : %+6.2f px   (want POSITIVE)\n', med_disp);
    fprintf('  SPREAD         disparity 10-90 pct      : %.2f .. %.2f px\n', ...
            prctile_local(du,10), prctile_local(du,90));

    ok_rect = abs(med_dv)   < 1.0;
    ok_sign = med_disp > 0;

    % --- Depth-based check ---------------------------------------------------
    slope = NaN; fx_meas = NaN; nz = 0;
    if ~isempty(D)
        % Depth comes from the RIGHT camera, so sample it at the RIGHT image
        % coordinates of each match.
        z = zeros(n,1);
        for k = 1:n
            ur = min(max(round(uu(k) - du(k)), 1), 640);
            vr = min(max(round(vv(k) + dv(k)), 1), 480);
            z(k) = double(D(vr, ur));
        end

        keep = isfinite(z) & z > 0.5 & z < 120 & du(:) > 0.5;
        nz   = sum(keep);
        if nz >= 10
            zk    = z(keep);
            dk    = du(keep);
            pred  = FX_PX * BASELINE_M ./ zk;         % what disparity SHOULD be
            slope = (pred(:)' * dk(:)) / (pred(:)' * pred(:));
            fx_meas = median(dk .* zk / BASELINE_M);  % fx, measured

            fprintf('\n  SCALE          measured/predicted slope : %.3f     (want ~1.00, %d pts)\n', slope, nz);
            fprintf('  FOCAL LENGTH   fx implied by geometry   : %.0f px  (HFOV %.1f deg)\n', ...
                    fx_meas, 2*atand(320/fx_meas));
        else
            fprintf('\n  SCALE: only %d points had usable depth - skipping.\n', nz);
        end
    end

    % --- Verdict -------------------------------------------------------------
    fprintf('\n  VERDICT:\n');
    report('rectified (vertical residual < 1 px)', ok_rect, ...
           'cameras have different Relative rotation - both must be [0, 5, 0]');
    report('disparity sign positive',              ok_sign, ...
           'left/right swapped - right camera Y must be NEGATIVE (ISO 8855: +Y is left)');
    if ~isnan(slope)
        ok_scale = abs(slope - 1) < 0.10;
        report('scale matches fx*b/z', ok_scale, ...
               sprintf(['baseline or fx is wrong. Either the Simulink Y offset is not %.3f m,\n' ...
                        '           or fx is not %g (fx implied here is %.0f)'], ...
                        BASELINE_M, FX_PX, fx_meas));
    else
        fprintf('    [ -- ] scale check skipped (no ground-truth depth)\n');
    end
    fprintf('\n');

    if p.Results.Plot
        figure('Name','check_stereo_geometry','NumberTitle','off');
        subplot(1,2,1);
        show_overlay(L, R);
        title('left (green) vs right (magenta) - offset should be horizontal only');
        subplot(1,2,2);
        histogram(du, 20);
        xlabel('disparity  u_{left} - u_{right}  [px]'); ylabel('matches');
        title(sprintf('median %.1f px', med_disp)); grid on;
    end

    out = struct('ok', ok_rect && ok_sign, 'n', n, ...
                 'median_vertical_px', med_dv, 'median_disparity_px', med_disp, ...
                 'slope', slope, 'fx_measured', fx_meas, 'n_depth', nz, ...
                 'baseline_m', BASELINE_M, 'fx_assumed', FX_PX);
end

% -----------------------------------------------------------------------------
function [L, R, D] = grab_pair()
% Grab both eyes as close together as possible, and confirm the left one did
% not change underneath us - that would mean the publisher timer fired in
% between and the pair is torn.

    for attempt = 1:5
        L1 = evalin('base','latest_frame');
        R1 = evalin('base','latest_frame_right');
        L2 = evalin('base','latest_frame');
        if isequal(L1, L2)
            L = L1; R = R1;
            D = [];
            if evalin('base','exist(''latest_depth_right'',''var'')')
                D = evalin('base','latest_depth_right');
                if ~isempty(D) && ndims(D) == 3, D = D(:,:,1); end
            end
            check_img(L, 'latest_frame');
            check_img(R, 'latest_frame_right');
            return;
        end
    end
    error(['check_stereo_geometry: could not grab a stable pair after 5 tries. ' ...
           'Pause the model (or stop the publisher timer) and run this again.']);
end

function check_img(im, name)
    if isempty(im) || ~isa(im,'uint8') || ndims(im) ~= 3 || ...
       size(im,1) ~= 480 || size(im,2) ~= 640 || size(im,3) ~= 3
        error('check_stereo_geometry: %s is not a valid 480x640x3 uint8 image.', name);
    end
end

% -----------------------------------------------------------------------------
function [du, dv, uu, vv, n] = match_grid(L, R)
% Block-match a grid of patches from the left image into the right image.
% Deliberately dependency-free: no Computer Vision or Image Processing Toolbox.

    A = gray_local(L);
    B = gray_local(R);

    HALF   = 10;                 % 21x21 patch
    DU_MIN = -96;  DU_MAX = 96;  % search BOTH ways so a swapped rig shows up
    DV_MAX = 4;                  % rectified pair => vertical travel is ~0

    us = 60:52:600;
    vs = 50:48:440;

    du = []; dv = []; uu = []; vv = [];
    for v = vs
        for u = us
            if u-HALF < 1 || u+HALF > 640 || v-HALF < 1 || v+HALF > 480, continue; end
            patch = A(v-HALF:v+HALF, u-HALF:u+HALF);
            if std(patch(:)) < 10, continue; end       % featureless sky/tarmac

            [off_u, off_v, ok] = best_match(patch, B, u, v, HALF, DU_MIN, DU_MAX, DV_MAX);
            if ~ok, continue; end

            % Reported disparity is u_left - u_right. off_u is where the patch
            % moved TO in the right image, so disparity is its negative.
            du(end+1,1) = -off_u; %#ok<AGROW>
            dv(end+1,1) =  off_v; %#ok<AGROW>
            uu(end+1,1) =  u;     %#ok<AGROW>
            vv(end+1,1) =  v;     %#ok<AGROW>
        end
    end
    n = numel(du);
end

function [off_u, off_v, ok] = best_match(patch, B, u, v, HALF, DU_MIN, DU_MAX, DV_MAX)
    off_u = 0; off_v = 0; ok = false;
    best = inf; second = inf;

    for dvv = -DV_MAX:DV_MAX
        vc = v + dvv;
        if vc-HALF < 1 || vc+HALF > 480, continue; end
        for duu = DU_MIN:DU_MAX
            uc = u + duu;
            if uc-HALF < 1 || uc+HALF > 640, continue; end
            cand = B(vc-HALF:vc+HALF, uc-HALF:uc+HALF);
            e = sum((cand(:) - patch(:)).^2);
            if e < best
                second = best; best = e; off_u = duu; off_v = dvv;
            elseif e < second
                second = e;
            end
        end
    end

    if ~isfinite(best) || best <= 0, return; end
    if second / best < 1.30, return; end   % ambiguous (repetitive texture)
    ok = true;

    % SUB-PIXEL REFINEMENT. The integer search above is only good to +-0.5 px.
    % At long range the true disparity is only a few px, so +-0.5 px is a ~17%
    % error - too coarse to check the baseline or measure fx. Fit a parabola
    % through the SSD at (best-1, best, best+1) along u and take its minimum.
    % Standard stereo practice; gets us to roughly +-0.1 px.
    uc = u + off_u;
    vc = v + off_v;
    if uc-HALF-1 >= 1 && uc+HALF+1 <= 640 && vc-HALF >= 1 && vc+HALF <= 480
        e0 = best;
        cm = B(vc-HALF:vc+HALF, uc-HALF-1:uc+HALF-1);
        cp = B(vc-HALF:vc+HALF, uc-HALF+1:uc+HALF+1);
        em = sum((cm(:) - patch(:)).^2);
        ep = sum((cp(:) - patch(:)).^2);
        denom = em - 2*e0 + ep;
        if denom > 0
            delta = 0.5 * (em - ep) / denom;
            if abs(delta) <= 1
                off_u = off_u + delta;
            end
        end
    end
end

% -----------------------------------------------------------------------------
function g = gray_local(im)
    im = double(im);
    g  = 0.299*im(:,:,1) + 0.587*im(:,:,2) + 0.114*im(:,:,3);
end

function show_overlay(A, B)
    C = zeros(480,640,3,'uint8');
    C(:,:,2) = uint8(gray_local(A));      % left  -> green
    C(:,:,1) = uint8(gray_local(B));      % right -> magenta
    C(:,:,3) = uint8(gray_local(B));
    image(C); axis image off;
end

function y = prctile_local(x, pct)
    xs = sort(x(:));
    idx = max(1, min(numel(xs), round(pct/100 * numel(xs))));
    y = xs(idx);
end

function report(label, ok, why)
    if ok
        fprintf('    [ OK ] %s\n', label);
    else
        fprintf('    [FAIL] %s\n           -> %s\n', label, why);
    end
end
