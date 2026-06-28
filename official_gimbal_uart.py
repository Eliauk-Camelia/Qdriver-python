#!/usr/bin/env python3
"""
QGimbal 上位机控制库 — 通过 UART / 蓝牙 SPP 发送二进制协议指令。

协议: 10 字节发送, 42 字节回复, 115200 8N1, CRC8(poly=0x07)
用法:
  python gimbal_uart.py /dev/ttyUSB0          # 交互模式
  python gimbal_uart.py /dev/ttyUSB0 enable   # 单条指令
"""

import struct
import serial
import sys
import time
from dataclasses import dataclass
from typing import Tuple

# ── CRC8 ──────────────────────────────────────────────
def _crc8(data: bytes, poly=0x07, init=0x00) -> int:
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ poly) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

def _make_packet(cmd: int, yaw: float = 0.0, pitch: float = 0.0) -> bytes:
    """构建 10 字节发送包: cmd(1) + yaw(f32 LE) + pitch(f32 LE) + crc8(1)"""
    data = struct.pack('<Bff', cmd, yaw, pitch)
    return data + struct.pack('<B', _crc8(data))

# ── 回复解析 ───────────────────────────────────────────
@dataclass
class GimbalStatus:
    enabled: bool
    stability: bool
    laser: bool
    imu_speed: Tuple[float, float]   # rpm (yaw, pitch)
    imu_angle: Tuple[float, float]   # rad
    motor_current: Tuple[float, float]  # A
    motor_speed: Tuple[float, float]    # rpm
    motor_angle: Tuple[float, float]    # rad

    def __str__(self):
        return (
            f"en={int(self.enabled)} stb={int(self.stability)} las={int(self.laser)} "
            f"imu_spd=({self.imu_speed[0]:.2f},{self.imu_speed[1]:.2f})rpm "
            f"imu_ang=({self.imu_angle[0]:.3f},{self.imu_angle[1]:.3f})rad "
            f"cur=({self.motor_current[0]:.2f},{self.motor_current[1]:.2f})A "
            f"spd=({self.motor_speed[0]:.1f},{self.motor_speed[1]:.1f})rpm "
            f"ang=({self.motor_angle[0]:.3f},{self.motor_angle[1]:.3f})rad"
        )

def _parse_response(data: bytes) -> GimbalStatus:
    """解析 42 字节回复包"""
    if len(data) < 42:
        raise ValueError(f"Response too short: {len(data)} bytes")
    # 验证 CRC8
    crc = _crc8(data[:41])
    if crc != data[41]:
        raise ValueError(f"CRC mismatch: calc={crc:02X} recv={data[41]:02X}")
    status = data[0]
    fields = struct.unpack('<ffffffffff', data[1:41])
    return GimbalStatus(
        enabled=bool(status & 0x01),
        stability=bool(status & 0x02),
        laser=bool(status & 0x04),
        imu_speed=(fields[0], fields[1]),
        imu_angle=(fields[2], fields[3]),
        motor_current=(fields[4], fields[5]),
        motor_speed=(fields[6], fields[7]),
        motor_angle=(fields[8], fields[9]),
    )

# ── 指令集 ─────────────────────────────────────────────
class Gimbal:
    """云台上位机控制接口"""

    CMD = {
        'nop': 0x00,
        'enable': 0x01,
        'disable': 0x02,
        'current': 0x03,
        'speed': 0x04,
        'angle': 0x05,
        'lowspeed': 0x06,
        'step': 0x07,
        'reset_imu': 0xFB,
        'laser_off': 0xFC,
        'laser_on': 0xFD,
        'stability_off': 0xFE,
        'stability_on': 0xFF,
    }

    def __init__(self, port: str, baud=115200, timeout=0.2):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(0.1)

    def close(self):
        self.ser.close()

    def _send(self, cmd: int, yaw=0.0, pitch=0.0) -> GimbalStatus:
        pkt = _make_packet(cmd, yaw, pitch)
        self.ser.write(pkt)
        self.ser.flush()
        resp = self.ser.read(42)
        return _parse_response(resp)

    # ── 开关 ──
    def enable(self)          -> GimbalStatus: return self._send(self.CMD['enable'])
    def disable(self)         -> GimbalStatus: return self._send(self.CMD['disable'])
    def stability_on(self)    -> GimbalStatus: return self._send(self.CMD['stability_on'])
    def stability_off(self)   -> GimbalStatus: return self._send(self.CMD['stability_off'])
    def laser_on(self)        -> GimbalStatus: return self._send(self.CMD['laser_on'])
    def laser_off(self)       -> GimbalStatus: return self._send(self.CMD['laser_off'])
    def reset_imu(self)       -> GimbalStatus: return self._send(self.CMD['reset_imu'])

    # ── 控制 ──
    def angle(self, yaw=0.0, pitch=0.0)    -> GimbalStatus: return self._send(self.CMD['angle'], yaw, pitch)
    def speed(self, yaw=0.0, pitch=0.0)    -> GimbalStatus: return self._send(self.CMD['speed'], yaw, pitch)
    def current(self, yaw=0.0, pitch=0.0)  -> GimbalStatus: return self._send(self.CMD['current'], yaw, pitch)
    def lowspeed(self, yaw=0.0, pitch=0.0) -> GimbalStatus: return self._send(self.CMD['lowspeed'], yaw, pitch)
    def step(self, yaw=0.0, pitch=0.0)     -> GimbalStatus: return self._send(self.CMD['step'], yaw, pitch)
    def nop(self)                          -> GimbalStatus: return self._send(self.CMD['nop'])

# ── 交互式 / CLI ───────────────────────────────────────
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='QGimbal 上位机控制')
    parser.add_argument('port', help='串口设备, 如 /dev/ttyUSB0')
    parser.add_argument('cmd', nargs='?', help='指令: enable, disable, angle Y P, speed Y P, ...')
    parser.add_argument('args', nargs='*', type=float, help='参数')
    parser.add_argument('-b', '--baud', type=int, default=115200, help='波特率 (默认 115200)')
    args = parser.parse_args()

    g = Gimbal(args.port, args.baud)

    if args.cmd is None:
        # 交互模式
        print("QGimbal 上位机 — 输入 help 查看指令")
        while True:
            try:
                line = input("> ").strip()
                if not line:
                    continue
                parts = line.split()
                cmd = parts[0].lower()
                vals = [float(x) for x in parts[1:3]] if len(parts) > 1 else []

                if cmd == 'help':
                    print("  enable/disable  stability_on/off  laser_on/off  reset_imu")
                    print("  nop             (查询状态)")
                    print("  angle Y P       (角度控制, rad)")
                    print("  speed Y P       (速度控制, rpm)")
                    print("  current Y P     (电流控制, A)")
                    print("  quit            (退出)")
                elif cmd == 'quit':
                    break
                elif cmd in g.CMD:
                    s = g._send(g.CMD[cmd], *vals[:2])
                    print(s)
                else:
                    print(f"未知指令: {cmd}")
            except (EOFError, KeyboardInterrupt):
                break
            except Exception as e:
                print(f"错误: {e}")
    else:
        # 单条指令
        cmd = args.cmd.lower()
        vals = args.args[:2] if args.args else []
        if cmd not in g.CMD:
            print(f"未知指令: {cmd}")
            sys.exit(1)
        s = g._send(g.CMD[cmd], *vals)
        print(s)

    g.close()
