#!/usr/bin/env bash
# orb2_rgbd_test_trial.sh OPTION [HZ]   -- ONE ORB-SLAM2 RGB-D HIL test trial on Yasser5G (2026-10-02).
#   OPTION 1 : gray, exactly like the RTAB-Map RGB-D milestone: MATLAB sends mono8 at 20 Hz,
#              ORB-SLAM2 reads /ovcam  (matrix config only_orbslam2_rgbd_oracle_nopin)
#   OPTION 2 : color at 20 Hz, ORB-SLAM2 reads MATLAB bgr8 directly (only_orbslam2_rgbd_color_oracle_nopin)
#   OPTION 3 : color at HZ (default 10), same configs as option 2
# Rules (Arian, 2026-10-02): VALID, >= 20 poses, Pi-side pacing >= 0.9, on Yasser5G. Plus his own review.
# Prints the rule table and the PC's Wi-Fi bytes sent (2-copies question). Never starts an n=10.
# Run from Git Bash:  bash scripts/hil_matrix/orb2_rgbd_test_trial.sh 1
set -uo pipefail
OPT="${1:?usage: orb2_rgbd_test_trial.sh 1|2|3 [HZ]}"; HZ="${2:-10}"
cd "$(dirname "$0")/../.."
case "$OPT" in
  1) CFG=only_orbslam2_rgbd_oracle_nopin;       TAG=RGBD_ORB2_GRAY_TEST;      export RGBD_COLOUR_MONO=1 RGBD_PUBLISH_HZ=20 ;;
  2) CFG=only_orbslam2_rgbd_color_oracle_nopin; TAG=RGBD_ORB2_COLOR20_TEST;   export RGBD_COLOUR_MONO=0 RGBD_PUBLISH_HZ=20 ;;
  3) CFG=only_orbslam2_rgbd_color_oracle_nopin; TAG=RGBD_ORB2_COLOR${HZ}_TEST; export RGBD_COLOUR_MONO=0 RGBD_PUBLISH_HZ="$HZ" ;;
  *) echo "OPTION must be 1, 2 or 3"; exit 2 ;;
esac
export RGBD_DROP_DUP_STAMPS=0
LOG="scripts/hil_matrix/logs/orb2_rgbd_test_${TAG}_$(date +%Y%m%d_%H%M%S).log"

netsh wlan show interfaces | grep -qE "^\s+SSID\s+:\s+Yasser5G" || { netsh wlan connect name=Yasser5G ssid=Yasser5G; sleep 15; }
netsh wlan show interfaces | grep -E " SSID|Signal" | tr -s ' ' | tr '\n' ' '; echo
netsh wlan show interfaces | grep -qE "^\s+SSID\s+:\s+Yasser5G" || { echo "RULE FAIL: not on Yasser5G"; exit 3; }

echo "=== option $OPT: config $CFG, tag $TAG, RGBD_COLOUR_MONO=$RGBD_COLOUR_MONO, RGBD_PUBLISH_HZ=$RGBD_PUBLISH_HZ  ($(date +%T))"
b0=$(powershell -NoProfile -Command "(Get-NetAdapterStatistics -Name Wi-Fi).SentBytes"); t0=$(date +%s)
py scripts/hil_matrix/run_matrix_rgbd.py --config "$CFG" --trials 1 --run-tag "$TAG" > "$LOG" 2>&1; rc=$?
b1=$(powershell -NoProfile -Command "(Get-NetAdapterStatistics -Name Wi-Fi).SentBytes"); t1=$(date +%s)
echo "orchestrator exit=$rc  log: $LOG"
grep -E "FAIL|VALID|APE RMSE|input Hz|pose Hz|track ms|init s" "$LOG"
echo "PC Wi-Fi sent: $(( (b1-b0)/1000000 )) MB over $(( t1-t0 )) s (whole trial incl. setup/drain)"

ssh -o ConnectTimeout=8 -o BatchMode=yes amaraly@192.168.1.60 "cd ~/ROS2-slam-hil && python3 - '$TAG' <<'EOF'
import sys, glob, csv, numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
ape = {}
for f in glob.glob('results/matrix_20*/results.csv'):
    for r in csv.DictReader(open(f)): ape[r['bag']] = r
for d in sorted(glob.glob('bags/*%s_orbslam2_rgbd_nopin*/bag' % sys.argv[1])):
    hb = []
    with AnyReader([Path(d)]) as rd:
        for c, t, raw in rd.messages(connections=[c for c in rd.connections if c.topic == '/sim/heartbeat']):
            hb.append((t / 1e9, rd.deserialize(raw, c.msgtype).data))
    hb = np.array(hb); pacing = np.polyfit(hb[:, 0], hb[:, 1], 1)[0] if len(hb) > 20 else float('nan')
    r = ape.get(d.split('/')[1], {}); st = r.get('status', 'missing'); poses = int(float(r.get('n_poses') or 0))
    rules = [('VALID', st == 'ok', st), ('poses >= 20', poses >= 20, poses), ('pacing >= 0.9', pacing >= 0.9, round(pacing, 3))]
    print('bag', d.split('/')[1])
    for name, ok, val in rules: print('  %-14s %-5s %s' % (name, 'PASS' if ok else 'FAIL', val))
    print('  APE RMSE %s m, input %s Hz, pose %s Hz' % (r.get('ape_rmse', '')[:6], r.get('input_rate_hz', '')[:5], r.get('pose_rate_hz', '')[:5]))
    print('  RULES: %s' % ('ALL PASS (Arian reviews next)' if all(o for _, o, _ in rules) else 'FAIL'))
EOF"
