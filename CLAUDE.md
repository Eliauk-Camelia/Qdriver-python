# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

电赛云台视觉跟踪 — 三问逻辑的 Python 上位机。本仓库是 `QGimbal-Vision-master-dev` 的**单文件简化封装**，使用上游 `GimbalTracker` (PID + speed_ctrl) 进行云台控制。

详细任务逻辑说明见 `TASK_LOGIC.md`。

### 外部依赖

本仓库**不包含** `vision/` 和 `control/` 包，运行时从平行目录导入：

```python
_VISION_PATH = os.path.join(os.path.dirname(__file__), "..", "down", "QGimbal-Vision-master-dev")
```

必须确保 `../down/QGimbal-Vision-master-dev/` 存在。

## 运行

```bash
pip install -r ../down/QGimbal-Vision-master-dev/requirements.txt
python main.py  # 顶部 problem = 1/2/3 切换题目
```

Pi：`eualik@10.194.157.81`，代码路径 `~/Desktop/Qdriver-python/`。

```bash
# 同步
rsync -avz ~/Desktop/WorkSpace/Qdriver-python/ eualik@10.194.157.81:~/Desktop/Qdriver-python/
# Pi 上运行
DISPLAY=:0 python ~/Desktop/Qdriver-python/main.py
```

## 架构

```
摄像头 (固定，不在云台上) → cv2.flip(-1) 180°旋转
  → detect_black_rect()  自适应阈值 → 闭运算 → 评分选最优 centroid
  → 抗闪烁 + 假目标防护
  → 状态机 LOCKED/SCANNING
  → GimbalTracker (上游 PID: P+I+D, 误差归一化)
  → speed_ctrl → STM32
```

### 关键认知：摄像头不在云台上

云台转动**不影响**摄像头画面。这是**开环控制**：检测误差 → PID 算出 RPM → 云台跟过去，没有视觉闭环。这就是为什么 speed_ctrl 可以工作（不像闭环系统那样会冲过头），也是为什么之前 angle_ctrl 看起来"不动"（发了角度但画面不变）。

### 控制架构（当前方案：PID + speed_ctrl）

使用上游 `control/tracker_control.py` 的 `GimbalTracker`：

```
raw 中心 (cx,cy) → err_px → 归一化 err/(frame/2) → PID(P+I+D) → 归一化输出 × max_rpm → speed_ctrl
```

PID 配置（`main.py` 内 `ctrl_cfg`）：
```python
ControlConfig(
    max_rpm_yaw=10.0, max_rpm_pitch=10.0,
    invert_yaw=True, invert_pitch=False,
    yaw_pid=PIDConfig(kp=1.5, ki=0.15, kd=0.04, integral_limit=0.15, output_limit=1.0),
    pitch_pid=PIDConfig(kp=1.2, ki=0.10, kd=0.03, integral_limit=0.15, output_limit=1.0),
)
```

调参：改 `max_rpm`（速度上限）和 `kp`（响应强度）。不要改 `invert_*`（方向已验证）。

### Yaw 方向（已验证）

- **负 RPM = 右转，正 RPM = 左转**
- `invert_yaw=True`：err_x 正（目标在 flip 图像右侧）→ PID 正 → invert 翻转 → 负 RPM → 右转 ✓

### Pitch 方向（注意！）

Pitch 的方向历史上反复出问题，如果发现 pitch 跟反了，改 `invert_pitch`：
- `invert_pitch=False` → PID 输出直接作为 RPM
- `invert_pitch=True` → PID 输出取反

### 检测管线

```
BGR → Gray → GaussianBlur(5×5) → AdaptiveThresh(BINARY_INV, block=11, C=2)
    → MORPH_CLOSE(7×7, iter=2) → findContours(EXTERNAL) → 评分选最优 → centroid
```

- `SEARCH_RADIUS=150`：有上一帧位置时只搜附近 ±150px，距离加权评分
- `MIN_AREA=5000`
- `cv2.flip(frame, -1)` 180°旋转画面，调试时需注意坐标系

### 假目标锁死防护

检测连续 5 帧找不到真轮廓 → `prev_center = None` 强制全局搜索。防止检测锁死在错误位置（area=0 但 center 不为 None 的假目标）。

### 状态机

```
LOCKED: center != None → PID 跟踪
  center == None & lost < 3 → 抗闪烁复用
  center == None & lost ≥ 3 → 保持（不发命令）
  lost ≥ 15 → SCANNING

SCANNING: speed_ctrl(SCAN_RPM, 0) Yaw 旋转
  center != None → 恢复 LOCKED
```

## 调试工具

| 脚本 | 用途 |
|------|------|
| `debug_detect.py` | 纯检测，不控制云台，看 err 值 |
| `test_pid_trace.py` | 误差序列喂 PID，隔离控制回路 |
| `test_four_dir.py` | speed_ctrl 四方向测试 |
| `test_angle_dir.py` | angle_ctrl 四方向测试 |
| `tune_yaw.py` / `tune_pitch.py` | 单轴 P 控制调参 |
| `reset_gimbal.py` | 云台归零 |

## 下位机固件

`../down/QGimbal-master/` — STM32F407 (大疆 C 板)：
- `Gimbal.h:133`：`pitch_max = 1.22f` (≈70°，已从 0.5 改大)
- 串口：USART6，1152000 8N1，CRC8
- 编译：`cd ../down/QGimbal-master && cmake --preset Debug && cmake --build build/Debug`

## 失败的方案（别再试）

| 方案 | 现象 | 原因 |
|------|------|------|
| speed_ctrl + P-only | 永远振荡 | 无阻尼，一帧冲过头 |
| speed_ctrl + MIN_RPM | 永远不收敛 | 最小速度强制过冲 |
| angle_ctrl（直接换算，无 PID） | 卡死不动 / 方向混乱 | angle_ctrl 与 speed_ctrl 符号约定不同；开环无反馈 |
| angle_ctrl + 累加角度 | yaw 转到 -143° | 无限累加 |
| EMA 平滑进控制回路 | 正反馈 | 滞后 = 追过期位置 |

## 关联仓库

- `../down/QGimbal-Vision-master-dev/` — 上游：视觉检测 + PID 控制 + 串口协议
- `../down/QGimbal-master/` — STM32 下位机固件
