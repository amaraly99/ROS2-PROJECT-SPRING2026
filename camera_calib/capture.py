import cv2, os, time
from picamera2 import Picamera2

os.makedirs("calib_imgs", exist_ok=True)

cam = Picamera2()
config = cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
cam.configure(config)
cam.start()
time.sleep(2)

CHECKERBOARD = (9, 6)
flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS

count = 0
print("SPACE = save (only saves if corners found) | Q = quit")

while True:
    frame = cam.capture_array()
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCornersSB(gray, CHECKERBOARD, flags)

    display = frame_bgr.copy()
    if ret:
        cv2.drawChessboardCorners(display, CHECKERBOARD, corners, ret)
        cv2.putText(display, f"FOUND! SPACE to save ({count}/25)", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    else:
        cv2.putText(display, f"NOT FOUND ({count}/25)", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

    cv2.imshow("Corner Detection - SPACE to save, Q to quit", display)
    key = cv2.waitKey(1) & 0xFF

    if key == ord(' '):
        if ret:
            path = f"calib_imgs/img_{count:03d}.jpg"
            cv2.imwrite(path, frame_bgr)
            print(f"  ✓ Saved {path}")
            count += 1
        else:
            print("  ✗ Corners not found — reposition and try again")
    elif key == ord('q'):
        break

cam.stop()
cv2.destroyAllWindows()
print(f"\nDone! {count} images saved.")
