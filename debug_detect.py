"""纯检测调试 — 不控制云台，只看检测结果。按 q 退出。"""
import cv2
import numpy as np
import math
import time

CAMERA_IDX = 0
FRAME_W, FRAME_H = 640, 480
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
    best_contour = None
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
            best_contour = cnt
            best_center = (cx, cy)

    return best_center, best_contour, (gray, adaptive, closed)


def main():
    cap = cv2.VideoCapture(CAMERA_IDX, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.open(CAMERA_IDX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Pipeline", cv2.WINDOW_NORMAL)

    prev_center = None
    frame_count = 0

    print("纯检测模式 — 按 q 退出")
    print(f"{'帧':>5} {'raw_x':>5} {'raw_y':>5} {'err_x':>6} {'err_y':>6} {'area':>8} {'contours':>8}")
    print("-" * 55)

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue
        frame = cv2.flip(frame, -1)
        h, w = frame.shape[:2]

        center, contour, (gray, adaptive, closed) = detect_black_rect(frame, prev_center)
        frame_count += 1

        # 终端输出
        if center:
            cx, cy = center
            err_x = cx - w / 2
            err_y = cy - h / 2
            area = cv2.contourArea(contour) if contour is not None else 0
            contours_total = len(cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0])
            print(f"{frame_count:>5} {cx:>5} {cy:>5} {err_x:>6.0f} {err_y:>6.0f} {area:>8.0f} {contours_total:>8}")
            prev_center = center
        else:
            print(f"{frame_count:>5}  LOST")

        # 画面绘制
        disp = frame.copy()
        # 十字线标画面中心
        cv2.drawMarker(disp, (w // 2, h // 2), (255, 0, 0),
                       cv2.MARKER_CROSS, 20, 2)
        if center:
            cv2.circle(disp, center, 6, (0, 255, 0), -1)
            if contour is not None:
                cv2.drawContours(disp, [contour], -1, (0, 255, 0), 2)
            cv2.putText(disp, f"({cx},{cy}) err=({err_x:.0f},{err_y:.0f})",
                        (cx + 15, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(disp, f"F:{frame_count}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("Detection", disp)

        # 管线可视化
        def to_bgr(img):
            if len(img.shape) == 2:
                return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            return img
        row1 = np.hstack([to_bgr(gray), to_bgr(adaptive), to_bgr(closed)])
        h_dbg = min(row1.shape[0], 300)
        scale = h_dbg / row1.shape[0]
        row1 = cv2.resize(row1, (int(row1.shape[1] * scale), h_dbg))
        cw = row1.shape[1] // 3
        cv2.putText(row1, "Gray", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(row1, "Adaptive+INV", (cw + 5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(row1, "Closed", (cw * 2 + 5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imshow("Pipeline", row1)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
