import cv2
import numpy as np
import glob
import pickle

# ── CONFIGURE THESE ────────────────────────────────────────────────────
CHECKERBOARD  = (9, 6)
SQUARE_SIZE   = 25.0
IMG_W, IMG_H  = 640, 480
CAMERA_TOPIC  = "/ovcam/image_raw"

BODY_T_CAM = [
    1., 0., 0., 0.,
    0., 1., 0., 0.,
    0., 0., 1., 0.,
    0., 0., 0., 1.
]
# ──────────────────────────────────────────────────────────────────────

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

obj_points, img_points = [], []
images = sorted(glob.glob("calib_imgs/*.jpg"))
print(f"Found {len(images)} images\n")

gray = None
for fname in images:
    img  = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Use the superior Sector-Based method that is immune to window overlap
    ret, corners = cv2.findChessboardCornersSB(gray, CHECKERBOARD, flags)
    
    if ret:
        obj_points.append(objp.copy()) # .copy() prevents memory reference bugs!
        img_points.append(corners)     # SB corners are ALREADY sub-pixel accurate
        
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
        cv2.imshow("Check", img)
        cv2.waitKey(300)
        print(f"  ✓ {fname}")
    else:
        print(f"  ✗ {fname}")

cv2.destroyAllWindows()
print(f"\nUsable: {len(obj_points)}/{len(images)}")

if len(obj_points) < 10:
    print("ERROR: Need at least 10 good images! Recapture more.")
    exit()

# ── Calibrate ──────────────────────────────────────────────────────────
print("\nCalibrating...")
rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, (IMG_W, IMG_H), None, None
)

fx, fy = K[0,0], K[1,1]
cx, cy = K[0,2], K[1,2]
k1, k2 = D[0,0], D[0,1]
p1, p2 = D[0,2], D[0,3]

print(f"\nRMS reprojection error: {rms:.4f} px  (good if < 0.5)")
print(f"fx={fx:.6f}  fy={fy:.6f}")
print(f"cx={cx:.6f}  cy={cy:.6f}")
print(f"k1={k1:.8f}  k2={k2:.8f}")
print(f"p1={p1:.8f}  p2={p2:.8f}")

# Warn if k2 is suspicious
if abs(k2) > 0.5:
    print(f"\n⚠️  WARNING: k2={k2:.4f} is too high! Recapture with more aggressive tilts.")
else:
    print(f"\n✓ k2 looks good!")

# ── Save pickle ────────────────────────────────────────────────────────
with open("ov5647_calibration.pkl", "wb") as f:
    pickle.dump({"camera_matrix": K, "dist_coeffs": D, "rms": rms}, f)

# ── Write OV2SLAM YAML ─────────────────────────────────────────────────
ov2slam_config = f"""%YAML 1.0
---

#--------------------------------------------------------------------------------------------
# Camera Parameters  (OV5647 @ {IMG_W}x{IMG_H}  |  RMS: {rms:.4f} px)
#--------------------------------------------------------------------------------------------

Camera.topic_left: {CAMERA_TOPIC}
Camera.topic_right: /cam1/image_raw

Camera.model_left: pinhole
Camera.model_right: pinhole

Camera.left_nwidth:  {IMG_W}
Camera.left_nheight: {IMG_H}

Camera.right_nwidth:  752
Camera.right_nheight: 480

# Intrinsics
Camera.fxl: {fx:.6f}
Camera.fyl: {fy:.6f}
Camera.cxl: {cx:.6f}
Camera.cyl: {cy:.6f}

# Distortion (OpenCV: k1 k2 p1 p2)
Camera.k1l: {k1:.8f}
Camera.k2l: {k2:.8f}
Camera.p1l: {p1:.8f}
Camera.p2l: {p2:.8f}

# Right camera (EuRoC defaults - replace if using stereo)
Camera.fxr: 457.587
Camera.fyr: 456.134
Camera.cxr: 379.999
Camera.cyr: 255.238

Camera.k1r: -0.28368365
Camera.k2r:  0.07451284
Camera.p1r: -0.00010473
Camera.p2r: -3.555907e-05

# Extrinsic T_body_cam0 (identity - replace if you have IMU)
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [{', '.join(str(v) for v in BODY_T_CAM)}]

body_T_cam1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [0.0125552670891, -0.999755099723,  0.0182237714554, -0.0198435579556,
           0.999598781151,  0.0130119051815,  0.0251588363115,  0.0453689425024,
          -0.0253898008918,  0.0179005838253,  0.999517347078,  0.00786212447038,
           0., 0., 0., 1.]

#--------------------------------------------------------------------------------------------
# SLAM Parameters
#--------------------------------------------------------------------------------------------
debug: 0
log_timings: 0

mono: 1
stereo: 0

force_realtime: 1
slam_mode: 1

buse_loop_closer: 0
bdo_stereo_rect: 0
alpha: 0.

bdo_undist: 0
finit_parallax: 20.

use_shi_tomasi: 0
use_fast: 0
use_brief: 1
use_singlescale_detector: 1

nmaxdist: 35
nfast_th: 10
dmaxquality: 0.001

use_clahe: 1
fclahe_val: 3

do_klt: 1
klt_use_prior: 1
btrack_keyframetoframe: 0
nklt_win_size: 9
nklt_pyr_lvl: 3

nmax_iter: 30
fmax_px_precision: 0.01
fmax_fbklt_dist: 0.5
nklt_err: 30.

bdo_track_localmap: 1
fmax_desc_dist: 0.2
fmax_proj_pxdist: 2.

doepipolar: 1
dop3p: 0
bdo_random: 1
nransac_iter: 100
fransac_err: 3.

fmax_reproj_err: 3.
buse_inv_depth: 1

robust_mono_th: 5.9915

use_sparse_schur: 1
use_dogleg: 0
use_subspace_dogleg: 0
use_nonmonotic_step: 0

apply_l2_after_robust: 1
nmin_covscore: 25
fkf_filtering_ratio: 0.95
do_full_ba: 0
"""

with open("ov5647_ov2slam.yaml", "w") as f:
    f.write(ov2slam_config)

print(f"\n✓ Saved: ov5647_ov2slam.yaml")
print(f"✓ Saved: ov5647_calibration.pkl")
