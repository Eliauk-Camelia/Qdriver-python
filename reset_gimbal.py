"""云台归零 — 发一次 angle_ctrl(0,0) 然后退出。"""
import os
import sys
_VISION_PATH = os.path.join(os.path.dirname(__file__), "..", "down", "QGimbal-Vision-master-dev")
sys.path.insert(0, _VISION_PATH)
from control.serial_stub import GimbalSerial
import time

SERIAL_PORT = "/dev/ttyAMA0"

gimbal = GimbalSerial(port=SERIAL_PORT, baudrate=1152000)
gimbal.open()
gimbal.enable()
print("归零中...")
gimbal.angle_ctrl(0.0, 0.0)
time.sleep(1.5)
print("归零完成")
gimbal.close()
