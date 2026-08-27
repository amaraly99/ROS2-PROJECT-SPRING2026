#!/usr/bin/env python3
"""Show what the FAST detector actually finds, CLAHE-off vs CLAHE-on.
This measures FEATURES (what SLAM uses), not pixels. Same frame, same detector.
"""
import sys, numpy as np, cv2
from rosbags.highlevel import AnyReader
from pathlib import Path

bag, out = sys.argv[1], sys.argv[2]
gray = None
with AnyReader([Path(bag)]) as r:
    conns = [c for c in r.connections if 'image' in c.topic.lower()]
    for c, ts, raw in r.messages(connections=conns):
        m = r.deserialize(raw, c.msgtype)
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY); break

clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
enh = clahe.apply(gray)

# OV2SLAM fast uses OpenCV FAST. Default threshold 20 (typical). Show both.
fast = cv2.FastFeatureDetector_create(threshold=20, nonmaxSuppression=True)
def detect_draw(im, txt):
    kp = fast.detect(im, None)
    vis = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
    for k in kp:
        cv2.circle(vis, (int(k.pt[0]), int(k.pt[1])), 2, (0, 255, 0), -1)
    cv2.rectangle(vis, (0, 0), (vis.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(vis, "%s: %d FAST corners" % (txt, len(kp)), (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
    return vis, len(kp)

l, nl = detect_draw(gray, "CLAHE OFF")
r_, nr = detect_draw(enh, "CLAHE ON")
gap = np.full((l.shape[0], 6, 3), 60, np.uint8)
cv2.imwrite(out, np.hstack([l, gap, r_]))
print("FAST corners  off=%d  on=%d  ratio=%.2fx" % (nl, nr, nr / max(nl, 1)))
