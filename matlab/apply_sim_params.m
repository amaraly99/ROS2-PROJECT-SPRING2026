function out = apply_sim_params(src, dst, start)
%APPLY_SIM_PARAMS  Copy a HIL model, set the shared camera and start pose, save, print sha256.
%
%   out = apply_sim_params(src, dst)            % default start: 20 m from the frozen sign
%   out = apply_sim_params(src, dst, start)     % start = struct with fields x, y, z, yaw (strings)
%
%   src    path to an existing .slx (hil_closed_loop.slx or any hil_closed_loop_baseline_*.slx)
%   dst    new model name, no extension, e.g. 'hil_closed_loop_shared_v1'. Saved next to src.
%   start  optional; the default is the 20 m start on the current bearing to the frozen sign.
%          For 30 m:  struct('x','5.13','y','4.30','z','10','yaw','2*pi-0.0460')
%          For 33 m:  struct('x','2.14','y','4.44','z','10','yaw','2*pi-0.0461')
%
% What it changes (and nothing else):
%   <mdl>/Simulation 3D Camera        FocalLength      -> [554, 554]
%   <mdl>/Simulation 3D Camera Right  FocalLength      -> [554, 554]   (only if the block exists)
%   <mdl>/x_integrator                InitialCondition -> start.x
%   <mdl>/y_integrator                InitialCondition -> start.y
%   <mdl>/z_integrator                InitialCondition -> start.z
%   <mdl>/yaw_integrator              InitialCondition -> start.yaw
% It asserts, without changing them, that OpticalCenter = [320, 240] and ImageSize = [480, 640]
% on every camera. It does not touch Commented flags, stereo baseline, depth ports, StopTime or the scene.
%
% src is never modified: the file is copied first and only the copy is opened.
% Note: an .slx save writes timestamps and a new UUID, so the sha256 is different on every save.
% Record the printed hash; do not expect to reproduce it by rerunning.

if nargin < 3 || isempty(start)
    start = struct('x', '15.12', 'y', '3.84', 'z', '10', 'yaw', '2*pi-0.0460');
end
FX = '[554, 554]';
CC = '[320, 240]';
SZ = '[480, 640]';

src = char(src);
assert(isfile(src), 'apply_sim_params: %s not found', src);
[folder, srcName] = fileparts(src);
dstFile = fullfile(folder, [dst '.slx']);
assert(~isfile(dstFile), 'apply_sim_params: %s exists; pick a new name', dstFile);
assert(~bdIsLoaded(dst), 'apply_sim_params: a model named %s is already loaded', dst);

copyfile(src, dstFile);
load_system(dstFile);
cleanup = onCleanup(@() close_system(dst, 0));

changes = {
    'Simulation 3D Camera',       'FocalLength',      FX
    'Simulation 3D Camera Right', 'FocalLength',      FX
    'x_integrator',               'InitialCondition', start.x
    'y_integrator',               'InitialCondition', start.y
    'z_integrator',               'InitialCondition', start.z
    'yaw_integrator',             'InitialCondition', start.yaw
};

fprintf('source %s  (sha256 %s)\n', src, sha256_file(src));
log = {};
for i = 1:size(changes, 1)
    blk = [dst '/' changes{i, 1}];
    if getSimulinkBlockHandle(blk) == -1
        if strcmp(changes{i, 1}, 'Simulation 3D Camera Right')
            fprintf('  skip  %-28s (block not in this model)\n', changes{i, 1});
            continue
        end
        error('apply_sim_params: block %s not found', blk);
    end
    old = get_param(blk, changes{i, 2});
    set_param(blk, changes{i, 2}, changes{i, 3});
    now_ = get_param(blk, changes{i, 2});
    assert(strcmp(now_, changes{i, 3}), 'apply_sim_params: %s %s did not take', blk, changes{i, 2});
    fprintf('  set   %-28s %-16s %-14s -> %s\n', changes{i, 1}, changes{i, 2}, old, now_);
    log(end+1, :) = {changes{i, 1}, changes{i, 2}, old, now_}; %#ok<AGROW>
end

for cam = {'Simulation 3D Camera', 'Simulation 3D Camera Right'}
    blk = [dst '/' cam{1}];
    if getSimulinkBlockHandle(blk) == -1, continue, end
    assert(strcmp(get_param(blk, 'OpticalCenter'), CC), '%s OpticalCenter is not %s', blk, CC);
    assert(strcmp(get_param(blk, 'ImageSize'), SZ), '%s ImageSize is not %s', blk, SZ);
end

save_system(dst);
clear cleanup   % closes the model
h = sha256_file(dstFile);
fprintf('saved  %s\nsha256 %s\n', dstFile, h);
out = struct('src', src, 'src_model', srcName, 'dst', dstFile, 'sha256', h, 'changes', {log});
end

function h = sha256_file(f)
fid = fopen(f, 'r');
assert(fid > 0, 'cannot open %s', f);
bytes = fread(fid, inf, '*uint8');
fclose(fid);
md = java.security.MessageDigest.getInstance('SHA-256');
d = typecast(md.digest(bytes), 'uint8');
h = lower(reshape(dec2hex(d, 2).', 1, []));
end
