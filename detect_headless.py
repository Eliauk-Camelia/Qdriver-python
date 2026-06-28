"""Headless detection debug — no GUI, just console output."""
import os, sys, math, time
import cv2, numpy as np
_VISION_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "down", "QGimbal-Vision-master-dev")
sys.path.insert(0, _VISION_PATH)
from vision.camera import CameraManager

MIN_AREA = 5000
SEARCH_RADIUS = 150

def detect_black_rect(frame_bgr, prev_center=None):
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    adaptive = cv2.adaptiveThreshold(blurred, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    kernel = np.ones((7, 7), np.uint8)
    closed = cv2.morphologyEx(adaptive, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_center = None
    best_score = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        if prev_center is not None:
            d = math.hypot(cx - prev_center[0], cy - prev_center[1])
            if d > SEARCH_RADIUS:
                continue
            score = area + (SEARCH_RADIUS - d) * 200
        else:
            score = area
        if score > best_score:
            best_score = score
            best_center = (cx, cy)
    return best_center

cam = CameraManager(index=0, width=640, height=480, fps=60)
cam.open()
w, h = 640, 480
print("Detection test — 5 seconds")
print(f"{'f':>5} {'cx':>5} {'cy':>5} {'err_x':>6} {'err_y':>6}")
prev = None
t0 = time.time()
fc = 0
while time.time() - t0 < 5:
    frame = cam.read()
    if frame is None:
        continue
    frame = cv2.flip(frame, -1)
    fc += 1
    center = detect_black_rect(frame, prev)
    if center:
        cx, cy = center
        print(f"{fc:>5} {cx:>5} {cy:>5} {cx-w/2:>6.0f} {cy-h/2:>6.0f}")
        prev = center
    else:
        print(f"{fc:>5}  LOST")
        prev = None
cam.release()
print("Done")
