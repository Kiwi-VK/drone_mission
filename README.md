# RPi 5 机载任务程序（SpeedyBee F405 V4 / ArduCopter）

这是面向四川省大学生低空技术创新挑战赛“无人机物流运输挑战赛”的机载任务程序。它把原来的 Mock 流程改成了带前置核验、实时局部占据栅格、统一速度仲裁、视觉闭环和证据日志的状态机。

## 先读这段

SpeedyBee F405 V4 当前如果是 Betaflight，程序会拒绝真实启动。此版本依赖 ArduCopter 的 MAVLink `GUIDED`、`LAND`、`LOCAL_POSITION_NED`、`EXTENDED_SYS_STATE` 和舵机命令。不能仅凭板卡型号判断固件兼容，也不会自动刷写飞控。

你提供的 CADDX 蜗牛 Nano AR0234 是 FPV 相机，MATEK GPS、TBS Unify、ELRS 和 SiK 数传也不会自动成为树莓派视频或室内定位输入。程序要求两个能被树莓派直接读取的视频源：下视相机用于投放点/降落区，前视相机用于固定障碍物目标图。FPV 发射机必须另接 USB 采集卡或改用树莓派可读的 CSI/USB 摄像头，才能满足这项要求。

MT-06 只有在确认协议、安装方向、单位和时间戳后才能接入测高。它的单点距离不能替代随机圆柱障碍物的左右空间观测；随机避障需要实时占据栅格输入。`GridReceiver` 只接受本机 `127.0.0.1:14660` 的新鲜栅格：`frame=takeoff_NED`、`coverage=full_airframe_height`，未知格视为不可通行，栅格过期就停车。

## 当前可运行的检查

在任意有 Python 3 的环境：

```bash
python3 acceptance_test.py
python3 -m compileall -q .
python3 mission.py --simulation
python3 mission.py --check-config
```

`--simulation` 只验证状态机和故障处理，不证明飞行性能。`--check-config` 在现场数据不完整时应失败；这是预期的安全行为。

## 现场配置顺序

1. 将 F405 V4 刷写并确认 ArduCopter；记录固件版本、MAVLink 串口、波特率、SERVO 输出通道和 ArduPilot failsafe 参数。
2. 先无桨台架验证：树莓派心跳、`LOCAL_POSITION_NED`、姿态、EKF、下视测高、舵机 PWM 和断链/手动接管。把结果写入 `mission_points.json` 的 `commissioning`，不能凭推测改为 `true`。
3. 用室内定位系统完成 LOCAL_POSITION_NED；GPS 只用于记录或辅助，不能在 5m 室内场地当作唯一闭环定位。
4. 将两个可采集视频源接入树莓派，完成相机内参、去畸变、下视图像到机体系矩阵和前视水平视场标定。
5. 现场采集投放点、目标图、降落标记模板，并填写 `mission_points.json` 中的本地 NED 坐标、5m 边界、固定障碍物位置和终点禁入区。
6. 投放机构必须提供装载/释放反馈文件。程序在释放前后都检查颜色、仓位和递增事件号，未确认不重试、不继续下一个点。
7. 完成两次单项低风险测试，再串联任务。任何相机、栅格、心跳、姿态、EKF、电池或地理边界异常都会停止任务；降落完成以飞控上锁和 `EXTENDED_SYS_STATE` 为准，退出不发送 RTL。

## 栅格输入格式

外部深度/激光节点每次向 UDP `127.0.0.1:14660` 发送一个 JSON 对象：

```json
{
  "sequence": 17,
  "sample_monotonic_s": 1234.56,
  "frame": "takeoff_NED",
  "coverage": "full_airframe_height",
  "resolution_m": 0.1,
  "origin_ne": [-3.0, -3.0],
  "width": 60,
  "height": 60,
  "cells": [0, 0, -1]
}
```

`cells` 必须有 `width*height` 个值：`0` 已观测空闲，`1` 障碍，`-1` 未知。未知不能当空闲。`sample_monotonic_s` 必须由同一树莓派时钟产生，超过 0.4 秒或序号倒退会被拒绝。

## 真实启动

确认 `mission.py --check-config` 没有输出错误后，才在无桨测试完成的情况下运行：

```bash
FC_PORT=/dev/serial/by-id/你的设备 \
FC_BAUD=115200 \
BOTTOM_CAMERA_PORT=/dev/video0 \
FRONT_CAMERA_PORT=/dev/video1 \
FRONT_HFOV_DEG=90 \
BATTERY_LOW_VOLTAGE=按电池实测填写 \
PAYLOAD_FEEDBACK_FILE=/run/drone/payload.json \
python3 mission.py --execute --confirm-real
```

不要在没有现场模板、坐标、相机标定、室内定位、实时栅格和投放反馈时绕过检查。项目不把原来的空坐标和模拟日志当作比赛成绩证据。
