# Qdriver-python

电赛云台视觉跟踪 — 三问逻辑 Python 上位机。使用上游 `QGimbal-Vision-master-dev` 的矩形检测 + PID 控制，通过串口驱动 STM32 云台。

## 硬件

- 树莓派 + USB 摄像头（固定，不在云台上）
- STM32F407 大疆 C 板云台（`/dev/ttyAMA0`，1152000 8N1）

## 快速开始

```bash
# 安装依赖
pip install -r ../down/QGimbal-Vision-master-dev/requirements.txt

# 纯检测（不控制云台）
DISPLAY=:0 python main.py

# 跟踪 + 云台控制
bash run.sh
```

## 常用命令

```bash
# 同步代码到树莓派
rsync -avz ~/Desktop/WorkSpace/Qdriver-python/ eualik@10.194.157.81:~/Desktop/Qdriver-python/

# SSH 连接
ssh eualik@10.194.157.81

# 树莓派运行
DISPLAY=:0 python ~/Desktop/Qdriver-python/main.py --serial-port /dev/ttyAMA0

# 归零
python reset_gimbal.py

# 纯检测调试
DISPLAY=:0 python debug_detect.py
```

## PID 调参

修改 `main.py` 中 `ctrl_cfg` 的 `yaw_pid` / `pitch_pid`：

```python
PIDConfig(kp=2.0,   # P: 越大跟越快，太大会振荡
          ki=0.0,   # I: 消除稳态残差，太大过冲
          kd=0.0,   # D: 阻尼防过冲，30fps 下效果有限
          integral_limit=0.2,
          output_limit=1.0)
```

**调好了的标志**：Error Plot 窗口曲线快速归零不振荡，纸不动时 err 保持 ±5px 内。

## 文件说明

| 文件 | 用途 |
|------|------|
| `main.py` | 主程序：检测 + PID + 激光 |
| `gimbal_commands.py` | 云台控制函数封装（串口协议） |
| `debug_detect.py` | 纯检测调试，看矩形框和坐标 |
| `detect_headless.py` | 无 GUI 检测，SSH 调试用 |
| `reset_gimbal.py` | 一键归零 |
| `run.sh` | 一键启动脚本 |
| `main01.py` | 学习第一步：拍照存盘 |
| `official_gimbal_uart.py` | 官方 UART 交互式控制历程 |
| `official_test_cmd.py` | 官方指令 HEX 查看 |
