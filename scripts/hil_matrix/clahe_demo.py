#!/usr/bin/env python3
"""Read one HIL camera frame from a short bag and show it with/without CLAHE.

Usage: clahe_demo.py <bag_dir> <out.png>
CLAHE matches OV2SLAM's fast/accurate knob: clipLimit=fclahe_val=3, 8x8 tiles.
"""
import sys
import numpy as np
import cv2
from rosbags.highlevel import AnyReader
from pathlib import Path

bag, out = sys.argv[1], sys.argv[2]
frame = None
with AnyReader([Path(bag)]) as r:
    conns = [c for c in r.connections if 'image' in c.topic.lower()]
    for c, ts, raw in r.messages(connections=conns):
        m = r.deserialize(raw, c.msgtype)
        h, w, enc = m.height, m.width, m.encoding.lower()
        buf = np.frombuffer(m.data, dtype=np.uint8)
        if enc in ('rgb8', 'bgr8'):
            img = buf.reshape(h, w, 3)
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if enc == 'rgb8' else cv2.COLOR_BGR2GRAY)
        elif enc in ('mono8', '8uc1'):
            gray = buf.reshape(h, w)
        elif 'nv12' in enc or enc == 'yuv420':
            yuv = buf[:h * w].reshape(h, w)   # Y plane = luma = grayscale
            gray = yuv
        else:
            gray = buf[:h * w].reshape(h, w)
        frame = gray
        print('frame: %dx%d enc=%s' % (w, h, enc))
        break

if frame is None:
    sys.exit('no image frame found in bag')

clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
enhanced = clahe.apply(frame)

# side by side with labels + a shared crop zoom so texture is visible
def label(im, txt):
    im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(im, (0, 0), (im.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(im, txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return im

left = label(frame, 'CLAHE OFF  (fast profile, as shipped)')
right = label(enhanced, 'CLAHE ON  (clipLimit=3, 8x8 tiles)')
gap = np.full((left.shape[0], 6, 3), 60, np.uint8)
combo = np.hstack([left, gap, right])
cv2.imwrite(out, combo)
print('wrote', out, combo.shape)
