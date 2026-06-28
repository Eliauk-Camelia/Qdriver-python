#!/usr/bin/env python3
"""QGimbal 指令测试 — 打印每条指令的 HEX 字节，无需连接硬件。"""
import struct

def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

def make_packet(cmd: int, yaw=0.0, pitch=0.0) -> bytes:
    data = struct.pack('<Bff', cmd, yaw, pitch)
    return data + struct.pack('<B', crc8(data))

CMDS = {
    'nop':           0x00,  'enable':        0x01,  'disable':      0x02,
    'current':       0x03,  'speed':         0x04,  'angle':        0x05,
    'lowspeed':      0x06,  'step':          0x07,
    'reset_imu':     0xFB,  'laser_off':     0xFC,  'laser_on':    0xFD,
    'stability_off': 0xFE,  'stability_on':  0xFF,
}

def show(cmd: int, yaw=0.0, pitch=0.0):
    pkt = make_packet(cmd, yaw, pitch)
    hex_str = ' '.join(f'{b:02X}' for b in pkt)
    print(f"  HEX: {hex_str}")
    print(f"   C : {' '.join(f'{b:3d}' for b in pkt)}")
    print()

print("=" * 55)
print("  QGimbal 指令测试 — 输入指令查看 HEX，输入 quit 退出")
print("=" * 55)
print()
print("  开关类: enable, disable, stability_on, stability_off,")
print("          laser_on, laser_off, reset_imu, nop")
print("  控制类: angle Y P,  speed Y P,  current Y P,  step Y P")
print("  示例:   angle 0.1 0")
print("          speed 50 -10")
print("          enable")
print()

while True:
    try:
        line = input("> ").strip()
    except (EOFError, KeyboardInterrupt):
        break
    if not line:
        continue
    if line == 'quit':
        break

    parts = line.split()
    name = parts[0].lower()
    vals = [float(x) for x in parts[1:3]] if len(parts) > 1 else []

    if name == 'help':
        print("  开关: enable disable stability_on/off laser_on/off reset_imu nop")
        print("  控制: angle/speed/current/step Y P")
        continue

    if name not in CMDS:
        print(f"  未知指令: {name}\n")
        continue

    cmd = CMDS[name]
    yaw = vals[0] if len(vals) > 0 else 0.0
    pitch = vals[1] if len(vals) > 1 else 0.0
    print(f"  指令: {name}  cmd=0x{cmd:02X}  yaw={yaw}  pitch={pitch}")
    show(cmd, yaw, pitch)
