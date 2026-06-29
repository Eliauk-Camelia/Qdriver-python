# 摄像头读取并显示画面
# 使用: python main.py --camera 0

"""
简单的摄像头预览脚本（使用 OpenCV）。
参数：
  --camera        摄像头索引（默认 0）
  --display       是否显示图形化窗口（0/1，默认 1）
                 - 1：显示图像（现有效果），并叠加矩形与 FPS
                 - 0：不显示窗口，在终端输出 FPS + 检测到的矩形中心点坐标/面积
  --print-interval 终端输出间隔秒数（仅 --display 0 时生效，默认 0.5）

GUI 模式按 'q' 或 ESC 退出；无窗口模式请按 Ctrl+C 退出。
"""

import argparse
import time
import sys
import os
from collections import deque
from gimbal_commands import GimbalCommands

# 导入上游 QGimbal 视觉项目模块
_VISION_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "down", "QGimbal-Vision-master-dev")
sys.path.insert(0, _VISION_PATH)

import cv2
import numpy as np

from vision.rect_detect import detect_rectangles, draw_detected_rect
import math

from control.config import ControlConfig, PIDConfig
from control.tracker_control import GimbalTracker

DEFAULT_CAMERA = 0  # 摄像头索引（默认 0）
DEFAULT_WIDTH = 640  # 期望宽度
DEFAULT_HEIGHT = 480  # 期望高度
DEFAULT_FPS = 60  # 期望帧率（USB 摄像头通常最高 30-60fps）
DEFAULT_DISPLAY = 1
DEFAULT_PRINT_INTERVAL = 0.05

# 控制默认参数（可通过命令行覆盖）
DEFAULT_CONTROL_ENABLED = 1
DEFAULT_MAX_RPM = 30.0
DEFAULT_DEADBAND_PX = 0.0
DEFAULT_LOST_TIMEOUT_S = 0.4


def parse_args():
    p = argparse.ArgumentParser(description="OpenCV 摄像头显示示例")
    p.add_argument('--camera', type=int, default=DEFAULT_CAMERA, help=f'摄像头索引（默认 {DEFAULT_CAMERA}）')
    p.add_argument('--display', type=int, choices=[0, 1], default=DEFAULT_DISPLAY,
                   help=f'是否显示图形化窗口（0/1，默认 {DEFAULT_DISPLAY}）')
    p.add_argument('--print-interval', type=float, default=DEFAULT_PRINT_INTERVAL,
                   help=f'终端输出间隔秒数（仅 --display 0 生效，默认 {DEFAULT_PRINT_INTERVAL}）')

    # 控制相关
    p.add_argument('--control', type=int, choices=[0, 1], default=DEFAULT_CONTROL_ENABLED,
                   help=f'是否启用 PID 控制输出（0/1，默认 {DEFAULT_CONTROL_ENABLED}）')
    p.add_argument('--max-rpm', type=float, default=DEFAULT_MAX_RPM,
                   help=f'最大转速输出（RPM，默认 {DEFAULT_MAX_RPM}）')
    p.add_argument('--deadband-px', type=float, default=DEFAULT_DEADBAND_PX,
                   help=f'像素死区（默认 {DEFAULT_DEADBAND_PX}）')
    p.add_argument('--lost-timeout', type=float, default=DEFAULT_LOST_TIMEOUT_S,
                   help=f'丢目标超时后复位控制器的时间（秒，默认 {DEFAULT_LOST_TIMEOUT_S}）')

    # 串口相关（协议在 control/serial_stub.py 内实现）
    p.add_argument('--serial-port', type=str, default=None, help='串口端口号，例如 COM3；不填则不发送')
    p.add_argument('--serial-baud', type=int, default=1152000, help='串口波特率（默认 1152000）')

    return p.parse_args()


def main():
    args = parse_args()

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.open(args.camera)  # V4L2 失败回退默认后端
    if not cap.isOpened():
        print(f"无法打开摄像头索引 {args.camera}. 请检查设备或更换索引。")
        sys.exit(2)

    # 设置参数
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, DEFAULT_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, DEFAULT_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, DEFAULT_FPS)

    display = bool(args.display)

    # 控制器初始化（方向已验证：invert_yaw=True, invert_pitch=False）
    ctrl_cfg = ControlConfig(
        enabled=bool(args.control),
        deadband_px=float(args.deadband_px),
        lost_timeout_s=float(args.lost_timeout),
        max_rpm_yaw=float(args.max_rpm),
        max_rpm_pitch=float(args.max_rpm),
        invert_yaw=True,
        invert_pitch=False,

        # ═══════════════════════════════════════════════
        # 常用命令 (copy-paste 用)
        # ═══════════════════════════════════════════════
        # 同步到树莓派:
        #   rsync -avz ~/Desktop/WorkSpace/Qdriver-python/ eualik@10.194.157.81:~/Desktop/Qdriver-python/
        # SSH:
        #   ssh eualik@10.194.157.81
        # 树莓派运行:
        #   DISPLAY=:0 python ~/Desktop/Qdriver-python/main.py --serial-port /dev/ttyAMA0
        # 纯检测:
        #   DISPLAY=:0 python ~/Desktop/Qdriver-python/main.py
        # 归零:
        #   python ~/Desktop/Qdriver-python/reset_gimbal.py

        yaw_pid=PIDConfig(kp=4.21, ki=0, kd=0, integral_limit=0.2, output_limit=1.0),
        pitch_pid=PIDConfig(kp=4.22, ki=0, kd=0, integral_limit=0.2, output_limit=1.0),
    )
    tracker = GimbalTracker(ctrl_cfg)
    gimbal = GimbalCommands(port=args.serial_port, baudrate=int(args.serial_baud))
    gimbal.open()
    gimbal.home()

    win_name = f"Camera {args.camera}"
    if display:
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow("Error Plot", cv2.WINDOW_NORMAL)

    last_print = 0.0
    prev_time = time.time()
    fps = 0.0
    err_history_x = deque(maxlen=120)
    err_history_y = deque(maxlen=120)
    laser_on = False
    LASER_THRESHOLD = 15.0  # 误差小于此值开激光 (px)
    distery = 0         #历史上是否检测到过矩形？
    # 摄像头读取帧
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("无法从摄像头读取到帧，正在重试...")
                time.sleep(0.1)
                continue

            frame = cv2.flip(frame, -1)
            h, w = frame.shape[:2]
            #---------------------------------------------------------------------------------------------------
            # 对每帧执行矩形检测
            rects = detect_rectangles(frame, min_area_ratio=0.005, max_area_ratio=0.5, angle_tol=25.0)
            # 选择面积最大的矩形
            best = rects[0] if rects else None

            if best is not None:
                distery = 1
            # else:
            #     if distery == 1:
            #         gimbal.laser_off(); laser_on = False
            #         print("激光 关闭")
            #     distery = 0
            # 计算 FPS
            now = time.time()
            dt = now - prev_time
            prev_time = now
            if dt > 0:
                alpha = 0.98
                inst_fps = 1.0 / dt
                fps = alpha * fps + (1 - alpha) * inst_fps if fps > 0 else inst_fps

            # PID 控制
            target_center = best.center if best is not None else None
            ret, ctrl_out = tracker.update(frame_w=w, frame_h=h, target_center=target_center, dt=max(dt, 1e-6), now=now)
            if ret:
                gimbal.speed_ctrl(ctrl_out.yaw_rpm, ctrl_out.pitch_rpm)
            elif best is None:
                if distery == 1:
                    gimbal.stop()  # 丢目标立即停转，防 pitch 跑飞
                else:
                    gimbal.speed_ctrl(10, 0)

            # 激光：误差小于阈值打开，丢目标关闭
            if best is not None:
                err_dist = (ctrl_out.err_x_px**2 + ctrl_out.err_y_px**2)**0.5
                if err_dist < LASER_THRESHOLD and not laser_on:
                    gimbal.laser_on(); laser_on = True
                    print("激光 开启")
            # else:
            #     if laser_on:
            #         gimbal.laser_off(); laser_on = False
            #         print("激光 关闭")
            #---------------------------------------------------------------------------------------------------
            # 终端诊断输出
            if best and int(time.time() * 2) % 60 == 0:
                cx, cy = best.center
                print(f"[检测] cx={cx:.0f} cy={cy:.0f} err=({cx-w/2:.0f},{cy-h/2:.0f}) area={best.area:.0f} rpm=({ctrl_out.yaw_rpm:.1f},{ctrl_out.pitch_rpm:.1f})")
            elif not best and int(time.time() * 2) % 60 == 0:
                print(f"[检测] 无目标")

            # 显示
            if display:
                if best is not None:
                    draw_detected_rect(frame, best)

                # 画面中心点
                cv2.drawMarker(frame, (w // 2, h // 2), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2)

                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"err(px)=({ctrl_out.err_x_px:.0f},{ctrl_out.err_y_px:.0f}) rpm=({ctrl_out.yaw_rpm:.1f},{ctrl_out.pitch_rpm:.1f})",
                    (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )

                cv2.imshow(win_name, frame)

                # ── 误差曲线 (示波器) ──
                err_history_x.append(ctrl_out.err_x_px)
                err_history_y.append(ctrl_out.err_y_px)
                plot = np.zeros((180, 400, 3), dtype=np.uint8)
                cv2.line(plot, (0, 90), (400, 90), (60, 60, 60), 1)  # 零线
                for i in range(1, len(err_history_x)):
                    x1 = int((i-1) * 400 / 120)
                    x2 = int(i * 400 / 120)
                    y1 = int(90 - err_history_x[i-1] * 0.5)  # 缩放
                    y2 = int(90 - err_history_x[i] * 0.5)
                    cv2.line(plot, (x1, y1), (x2, y2), (255, 150, 50), 2)
                    y1 = int(90 - err_history_y[i-1] * 0.5)
                    y2 = int(90 - err_history_y[i] * 0.5)
                    cv2.line(plot, (x1, y1), (x2, y2), (50, 200, 255), 2)
                cv2.putText(plot, "err_x(橙)  err_y(蓝)", (5, 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                cv2.imshow("Error Plot", plot)

                key = cv2.waitKey(1) & 0xFF
                # 按 'q' 或 ESC 退出
                if key == ord('q') or key == 27:
                    break
            else:
                # 无窗口：终端输出 FPS + 检测结果（按间隔打印，避免刷屏）
                if args.print_interval <= 0 or (now - last_print) >= args.print_interval:
                    last_print = now
                    if best is None:
                        print(f"fps={fps:.1f} rect=none rpm=({ctrl_out.yaw_rpm:.1f},{ctrl_out.pitch_rpm:.1f})")
                    else:
                        cx, cy = best.center
                        area = best.area
                        print(
                            f"fps={fps:.1f} cx={cx:.1f} cy={cy:.1f} area={area:.0f} "
                            f"err=({ctrl_out.err_x_px:.0f},{ctrl_out.err_y_px:.0f}) rpm=({ctrl_out.yaw_rpm:.1f},{ctrl_out.pitch_rpm:.1f})"
                        )

    except KeyboardInterrupt:
        print('\n收到中断，退出...')
    finally:
        cap.release()
        gimbal.close()
        if display:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

# ═══════════════════════════════════════════════
# 常用命令 (copy-paste 用)
# ═══════════════════════════════════════════════
# 同步到树莓派:
#   rsync -avz ~/Desktop/WorkSpace/Qdriver-python/ eualik@10.194.157.81:~/Desktop/Qdriver-python/
# SSH:
#   ssh eualik@10.194.157.81
# 树莓派运行:
#   DISPLAY=:0 python ~/Desktop/Qdriver-python/main.py --serial-port /dev/ttyAMA0
# 纯检测:
#   DISPLAY=:0 python ~/Desktop/Qdriver-python/main.py
# 归零:
#   python ~/Desktop/Qdriver-python/reset_gimbal.py
