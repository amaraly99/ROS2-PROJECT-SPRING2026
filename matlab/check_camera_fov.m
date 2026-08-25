function out = check_camera_fov(mode)
%CHECK_CAMERA_FOV  Decide whether the Simulation 3D Camera block's "Focal
%                  length" parameter actually drives the rendered view.
%
% WHY THIS EXISTS
%   hil_closed_loop.slx has  FocalLength = [1200, 1200]  on the camera block,
%   but hil_sim_ov2slam.yaml, hil_ros_init_LT.m:34 and the paper's IBVS
%   stand-off equation (access.tex:1310) all assume fx = 554.
%
%       fx = 554   ->  HFOV = 2*atan(320/554)  = 60.0 deg
%       fx = 1200  ->  HFOV = 2*atan(320/1200) = 29.9 deg
%
%   Those are very different cameras. Everything downstream needs the right
%   one: SLAM infers motion from how fast pixels move, the controller infers
%   distance from how big the target's box looks, and stereo depth is
%   literally  fx * baseline / disparity.  Settle this before adding stereo.
%
%   The block's own dialog text says it "uses the focal length ... to model the
%   lens", and the Parameters tab has no field-of-view field at all - so there
%   is nothing else for the render FOV to come from. That points to 1200 being
%   live, but it is a strong hint, not proof. Hence this test.
%
% HOW IT WORKS
%   With no Pi connected, cmd_vel stays zero (hil_ros_init_LT.m:46-50) so the
%   drone hovers at its initial pose and the rendered view is static. Render
%   the same scene at two focal lengths, then test one specific hypothesis:
%
%     If focal length is live, the narrow (fx=1200) image is exactly the middle
%     of the wide (fx=554) image, magnified by 1200/554. A world point at
%     (u-cx) in the narrow image sits at (u-cx)*554/1200 in the wide one.
%
%   So we warp the wide image by that factor and see whether it becomes the
%   narrow image. High correlation after warping = focal length drives the
%   render. Two identical images to begin with = it does not.
%
%   (Naive block matching between the two would NOT work here: a zoom changes
%   the size of every patch, so patches stop matching precisely when the zoom
%   is real. The warp test does not have that problem.)
%
% USAGE
%   0) Run hil_ros_init_LT.  Do NOT start the Pi stack.
%   1) Open hil_closed_loop.slx and set Stop Time to 2.
%   2) Run the model.  When it stops:      check_camera_fov capture
%   3) Camera block -> Parameters tab -> Focal length = [554, 554].
%   4) Run the model again. When it stops: check_camera_fov capture
%   5)                                     check_camera_fov compare
%
%   Captures land in check_camera_fov_data.mat next to this script.
%   'reset' throws them away and starts over.
%
% See also CHECK_STEREO_GEOMETRY.

    if nargin < 1, mode = 'compare'; end
    out = struct();

    datafile = fullfile(fileparts(mfilename('fullpath')), 'check_camera_fov_data.mat');
    MODEL    = 'hil_closed_loop';
    CAMBLOCK = [MODEL '/Simulation 3D Camera'];

    switch lower(mode)
        case 'reset'
            if exist(datafile, 'file'), delete(datafile); end
            fprintf('check_camera_fov: captures cleared.\n');

        case 'capture'
            do_capture(datafile, CAMBLOCK);

        case 'compare'
            out = do_compare(datafile);

        otherwise
            error('check_camera_fov: mode must be ''capture'', ''compare'' or ''reset''.');
    end
end

% -----------------------------------------------------------------------------
function do_capture(datafile, camblock)

    frame = evalin('base', 'latest_frame');
    if isempty(frame) || ~isa(frame,'uint8') || ndims(frame) ~= 3 || ...
       size(frame,1) ~= 480 || size(frame,2) ~= 640
        error(['check_camera_fov: latest_frame is not a valid 480x640x3 uint8 ' ...
               'image. Run the model first and let it finish.']);
    end

    % Read the focal length straight off the block, so each capture labels
    % itself and you cannot mix up which is which.
    try
        fl_str = get_param(camblock, 'FocalLength');
    catch
        error(['check_camera_fov: cannot read %s. Is hil_closed_loop open, and ' ...
               'is the camera block still named "Simulation 3D Camera"?'], camblock);
    end
    fl = str2num(fl_str); %#ok<ST2NM>
    if isempty(fl)
        error('check_camera_fov: could not parse FocalLength "%s".', fl_str);
    end
    fx = fl(1);

    if exist(datafile, 'file')
        S = load(datafile);
        caps = S.caps;
    else
        caps = struct('fx', {}, 'frame', {});
    end

    % Replace an existing capture at the same fx rather than accumulating.
    idx = find([caps.fx] == fx, 1);
    if isempty(idx), idx = numel(caps) + 1; end
    caps(idx).fx    = fx;
    caps(idx).frame = frame;

    save(datafile, 'caps');

    fprintf('check_camera_fov: captured a frame at fx = %g  (would be HFOV %.1f deg)\n', ...
            fx, 2*atand(320/fx));
    fprintf('  captures on file: %s\n', mat2str([caps.fx]));
    if numel(caps) < 2
        fprintf('  Now change Focal length on the block, re-run, and capture again.\n');
    else
        fprintf('  Ready. Run:  check_camera_fov compare\n');
    end
end

% -----------------------------------------------------------------------------
function out = do_compare(datafile)

    if ~exist(datafile, 'file')
        error('check_camera_fov: no captures yet. Run ''check_camera_fov capture'' first.');
    end
    S = load(datafile);
    caps = S.caps;
    if numel(caps) < 2
        error(['check_camera_fov: only %d capture(s) (fx = %s). Change the block''s ' ...
               'Focal length, re-run the model, and capture again.'], ...
               numel(caps), mat2str([caps.fx]));
    end

    [~, ord] = sort([caps.fx]);
    wide   = caps(ord(1));       % smaller fx = sees more
    narrow = caps(ord(end));     % larger  fx = zoomed in

    fprintf('\n=== check_camera_fov ===\n');
    fprintf('  narrow capture: fx = %-6g  (would be HFOV %.1f deg)\n', ...
            narrow.fx, 2*atand(320/narrow.fx));
    fprintf('  wide   capture: fx = %-6g  (would be HFOV %.1f deg)\n', ...
            wide.fx,   2*atand(320/wide.fx));

    identical = isequal(narrow.frame, wide.frame);
    diffimg   = double(narrow.frame) - double(wide.frame);
    mad       = mean(abs(diffimg(:)));

    % The hypothesis test: warp the wide image as if focal length were live,
    % and see whether it turns into the narrow image.
    [r_warp, r_raw, warped] = zoom_hypothesis(narrow.frame, wide.frame, narrow.fx, wide.fx);

    fprintf('\n  pixel-identical                  : %s  (mean abs diff %.2f / 255)\n', ...
            string(identical), mad);
    fprintf('  correlation, wide vs narrow      : %+.3f   <- high means nothing changed\n', r_raw);
    fprintf('  correlation, WARPED wide vs narrow: %+.3f   <- high means focal length is live\n', r_warp);

    figure('Name','check_camera_fov','NumberTitle','off');
    subplot(1,3,1); imshow(narrow.frame); title(sprintf('as rendered, fx = %g', narrow.fx));
    subplot(1,3,2); imshow(wide.frame);   title(sprintf('as rendered, fx = %g', wide.fx));
    subplot(1,3,3); imshow(uint8(warped)); title(sprintf('fx=%g warped by %.3f', wide.fx, wide.fx/narrow.fx));

    fprintf('\n  VERDICT: ');
    if identical || mad < 0.5
        verdict = 'inert';
        fprintf('Focal length is INERT metadata.\n');
        fprintf(['    Changing it did not change the render, so fx = 554 was correctly\n' ...
                 '    derived from the real 60 deg FOV and everything downstream is fine.\n\n' ...
                 '    ACTION: set the block to [554, 554] so the model stops contradicting\n' ...
                 '    the calibration, then carry on with stereo. Nothing published changes.\n']);

    elseif r_warp > 0.70 && r_warp > r_raw + 0.10
        verdict = 'live';
        fprintf('Focal length DRIVES the render.\n');
        fprintf(['    Warping the wide image by exactly the focal-length ratio turns it into\n' ...
                 '    the narrow image, so the block really is rendering an fx = %g camera\n' ...
                 '    (HFOV %.1f deg) while SLAM, the controller and the paper were all told\n' ...
                 '    554 (60 deg).\n\n' ...
                 '    ACTION: put the block BACK to [%g, %g]. Do NOT "fix" it to 554 - this\n' ...
                 '    model is what produced every published result, and changing the camera\n' ...
                 '    would invalidate all of it. Fix the DESCRIPTIONS instead:\n' ...
                 '      * fxl/fyl -> %g in hil_sim_ov2slam.yaml and the stereo YAML\n' ...
                 '      * access.tex:1310 stand-off -> %g*1.5/(0.55*480) = %.2f m, not 3.15 m\n' ...
                 '      * the fx=554 comment at hil_ros_init_LT.m:34\n' ...
                 '      * check_stereo_geometry(''Fx'', %g) from now on\n' ...
                 '      * tell the Pi Claude - it changes the accuracy write-up too\n'], ...
                 narrow.fx, 2*atand(320/narrow.fx), narrow.fx, narrow.fx, ...
                 narrow.fx, narrow.fx, narrow.fx*1.5/(0.55*480), narrow.fx);

    else
        verdict = 'ambiguous';
        fprintf('AMBIGUOUS - do not guess.\n');
        fprintf(['    The two renders differ, but warping does not reconcile them\n' ...
                 '    (warped %.3f vs raw %.3f). Most likely the drone was not at the same\n' ...
                 '    pose in both runs, or the scene moved between them.\n' ...
                 '    Re-run: check_camera_fov reset, confirm the Pi is disconnected and\n' ...
                 '    Stop Time is identical for both runs, then capture both again.\n'], ...
                 r_warp, r_raw);
    end
    fprintf('\n');

    out = struct('verdict', verdict, 'fx_narrow', narrow.fx, 'fx_wide', wide.fx, ...
                 'r_warp', r_warp, 'r_raw', r_raw, 'mean_abs_diff', mad, ...
                 'identical', identical);
end

% -----------------------------------------------------------------------------
function [r_warp, r_raw, warped] = zoom_hypothesis(narrowImg, wideImg, fx_n, fx_w)
% If focal length is live then, for a world point,
%     (u_wide - cx) = (u_narrow - cx) * fx_w/fx_n
% so sampling the wide image at those scaled coordinates should reproduce the
% narrow image. Everything here is base MATLAB - no toolboxes.

    A = gray_local(narrowImg);
    B = gray_local(wideImg);

    cx = 320; cy = 240;           % the block's Optical center
    s  = fx_w / fx_n;             % < 1

    [U, V]   = meshgrid(1:640, 1:480);
    Us = cx + (U - cx) * s;
    Vs = cy + (V - cy) * s;

    warped = interp2(U, V, B, Us, Vs, 'linear', NaN);
    m = isfinite(warped);

    r_warp = corr_local(A(m),   warped(m));
    r_raw  = corr_local(A(:),   B(:));

    warped(~m) = 0;
end

function r = corr_local(x, y)
% Pearson correlation without the Statistics Toolbox.
    x = double(x(:)); y = double(y(:));
    x = x - mean(x);  y = y - mean(y);
    d = sqrt(sum(x.^2) * sum(y.^2));
    if d == 0
        r = NaN;
    else
        r = sum(x .* y) / d;
    end
end

function g = gray_local(im)
    im = double(im);
    g  = 0.299*im(:,:,1) + 0.587*im(:,:,2) + 0.114*im(:,:,3);
end
