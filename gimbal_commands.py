"""云台控制命令 — 所有 GimbalSerial 函数的语义化封装。

每个函数只做一件事，参数和返回值都有中文注释。
底层通过上游 control/serial_stub.py 的 GimbalSerial 与 STM32 通信。

方向约定（2026-06-28 验证）：
  Yaw:   负 RPM / 负角度 = 右转    正 RPM / 正角度 = 左转
  Pitch: 正 RPM = 下俯            负 RPM = 上仰
         angle_ctrl 的 pitch 符号与 speed_ctrl 相反！

使用示例：
  g = GimbalCommands("/dev/ttyAMA0")
  g.open()
  g.home()                        # 归零
  g.track_speed(yaw_rpm, pitch_rpm)  # 速度跟踪
  g.laser_on()
  g.close()
"""

from __future__ import annotations
import os, sys, time

_VISION_PATH = os.path.join(os.path.dirname(__file__), "..", "down", "QGimbal-Vision-master-dev")
sys.path.insert(0, _VISION_PATH)
from control.serial_stub import GimbalSerial


# ═══════════════════════════════════════════════════════════════════════
# 主类
# ═══════════════════════════════════════════════════════════════════════

class GimbalCommands:
    """云台控制命令集合。封装 GimbalSerial，提供语义化接口。"""

    def __init__(self, port: str = "/dev/ttyAMA0", baudrate: int = 1152000):
        self._g = GimbalSerial(port=port, baudrate=baudrate)
        self._laser = False

    # ── 连接 ──────────────────────────────────────────────────────────

    def open(self):
        """打开串口并使能电机。"""
        self._g.open()
        self._g.enable()

    def close(self):
        """关闭串口。"""
        self._g.close()

    # ── 归零 ──────────────────────────────────────────────────────────

    def home(self, wait: float = 1.5):
        """归零：发送 angle_ctrl(0,0)，等待云台转到零位。
        Args:
            wait: 等待时间（秒），默认 1.5s 足够云台归零。
        """
        self._g.angle_ctrl(0.0, 0.0)
        time.sleep(wait)

    # ── 电机使能/失能 ─────────────────────────────────────────────────

    def motor_on(self):
        """电机使能（上电）。"""
        self._g.enable()

    def motor_off(self):
        """电机失能（断电，云台自由转动）。"""
        self._g.disable()

    # ── 角度控制 ──────────────────────────────────────────────────────
    # 发送目标绝对角度，STM32 内部 angle PID 自行平滑到位。
    # 适合：开环指向，摄像头不在云台上的场景。
    # 注意：Pitch 符号约定 — 正角度 = 上仰，负角度 = 下俯（与 speed_ctrl 相反！）

    def angle_ctrl(self, yaw_rad: float, pitch_rad: float):
        """绝对角度控制。
        Args:
            yaw_rad:  目标 Yaw 角度 (rad)。0=正前方，负=左，正=右。
            pitch_rad: 目标 Pitch 角度 (rad)。0=水平，正=上仰，负=下俯。
        """
        self._g.angle_ctrl(yaw_rad, pitch_rad)

    def angle_step(self, yaw_delta_rad: float, pitch_delta_rad: float):
        """步进角度控制 — 在当前角度上累加 Δ。
        STM32 侧累加到 target_angle → angle PID。
        注意：每帧调用会持续累加，容易跑飞。一般用于单次步进。
        """
        self._g.step_angle_ctrl(yaw_delta_rad, pitch_delta_rad)

    # ── 速度控制 ──────────────────────────────────────────────────────
    # 发送目标 RPM，STM32 内部 speed PID 维持转速。
    # 适合：持续扫描、需要闭环跟踪的场景。
    # Pitch: 正 RPM = 下俯，负 RPM = 上仰

    def speed_ctrl(self, yaw_rpm: float, pitch_rpm: float):
        """速度控制（开环）。
        Args:
            yaw_rpm:   Yaw 轴目标转速 (RPM)。正=左转，负=右转。
            pitch_rpm: Pitch 轴目标转速 (RPM)。正=下俯，负=上仰。
        """
        self._g.speed_ctrl(yaw_rpm, pitch_rpm)

    def track_speed(self, yaw_rpm: float, pitch_rpm: float):
        """速度跟踪（同 speed_ctrl，语义化命名）。"""
        self.speed_ctrl(yaw_rpm, pitch_rpm)

    def scan(self, rpm: float = 10.0):
        """Yaw 轴匀速旋转扫描。Pitch 保持零。
        Args:
            rpm: 扫描转速 (RPM)，默认 10。
        """
        self._g.speed_ctrl(rpm, 0.0)

    def stop(self):
        """停止所有运动（发送 speed_ctrl(0,0)）。"""
        self._g.speed_ctrl(0.0, 0.0)

    # ── 电流/力矩控制 ─────────────────────────────────────────────────
    # 直接控制电机电流（力矩）。单位：A。
    # 适合：需要恒力矩输出或增稳模式的场景。

    def current_ctrl(self, yaw_amps: float, pitch_amps: float):
        """电流（力矩）控制。
        Args:
            yaw_amps:   Yaw 轴目标电流 (A)。
            pitch_amps: Pitch 轴目标电流 (A)。
        """
        self._g.current_ctrl(yaw_amps, pitch_amps)

    # ── 低速高转矩模式 ────────────────────────────────────────────────

    def low_speed_ctrl(self, yaw_rad_per_s: float, pitch_rad_per_s: float):
        """低速高转矩角度控制。target_angle 持续累加 → angle PID。
        Args:
            yaw_rad_per_s:   Yaw 角速度 (rad/s)。
            pitch_rad_per_s: Pitch 角速度 (rad/s)。
        """
        self._g.low_speed_ctrl(yaw_rad_per_s, pitch_rad_per_s)

    # ── 激光 ──────────────────────────────────────────────────────────

    def laser_on(self):
        """打开激光。"""
        self._g.enable_laser()
        self._laser = True

    def laser_off(self):
        """关闭激光。"""
        self._g.disable_laser()
        self._laser = False

    def laser_toggle(self):
        """切换激光状态。"""
        if self._laser:
            self.laser_off()
        else:
            self.laser_on()

    @property
    def laser(self) -> bool:
        """激光当前状态。"""
        return self._laser

    # ── 增稳 ──────────────────────────────────────────────────────────

    def stability_on(self):
        """开启 IMU 增稳。"""
        self._g.enable_stability()

    def stability_off(self):
        """关闭 IMU 增稳。"""
        self._g.disable_stability()

    # ── IMU ───────────────────────────────────────────────────────────

    def reset_imu(self):
        """重置 IMU 姿态（重新标定水平面）。"""
        self._g.reset_imu()

    # ── 查询 ──────────────────────────────────────────────────────────

    def query(self):
        """查询云台当前状态（角度、转速等），返回 GimbalFeedback 或 None。"""
        return self._g.query()

    # ── 底层访问 ──────────────────────────────────────────────────────

    @property
    def raw(self) -> GimbalSerial:
        """直接访问底层 GimbalSerial 对象，用于调用未封装的函数。"""
        return self._g


# ═══════════════════════════════════════════════════════════════════════
# 快捷函数（无需创建类实例，适合简单脚本）
# ═══════════════════════════════════════════════════════════════════════

def quick_home(port: str = "/dev/ttyAMA0"):
    """一键归零并关闭。"""
    g = GimbalSerial(port=port, baudrate=1152000)
    g.open()
    g.enable()
    g.angle_ctrl(0.0, 0.0)
    time.sleep(1.5)
    g.close()


def quick_test_direction(port: str = "/dev/ttyAMA0", rpm: float = 2.5, duration: float = 1.5):
    """快速四方向测试，观察云台方向是否正确。"""
    g = GimbalSerial(port=port, baudrate=1152000)
    g.open()
    g.enable()
    g.angle_ctrl(0.0, 0.0)
    time.sleep(1.5)
    for label, ry, rp in [("左转", -rpm, 0), ("右转", rpm, 0),
                            ("上仰", 0, -rpm), ("下俯", 0, rpm)]:
        print(f"【{label}】yaw={ry:+.1f} pitch={rp:+.1f}")
        g.speed_ctrl(ry, rp)
        time.sleep(duration)
        g.speed_ctrl(0.0, 0.0)
        time.sleep(0.5)
    g.angle_ctrl(0.0, 0.0)
    time.sleep(1.5)
    g.close()
    print("完成")
