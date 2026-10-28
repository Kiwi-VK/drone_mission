# 无屏比赛远程启动说明

## 第一次准备

树莓派 Ubuntu 上执行一次：

```bash
cd /home/kiwivk/drone_mission_project_rpi5_ready
chmod +x setup_remote_start_ubuntu.sh remote_start.sh remote_stop.sh remote_status.sh remote_logs.sh preflight_remote_check.sh
./setup_remote_start_ubuntu.sh
sudo reboot
```

## 比赛当天连接方式

树莓派没有屏幕时，让树莓派和你的电脑/手机处于同一个网络，例如：

1. 电脑或手机开热点，树莓派提前保存该 Wi-Fi；或
2. 比赛现场路由器；或
3. 网线直连/同一交换机。

在电脑上连接树莓派：

```bash
ssh kiwivk@<树莓派IP>
```

如果知道主机名可试：

```bash
ssh kiwivk@kiwivk.local
```

查看树莓派 IP 的常用方法：路由器后台、手机热点已连接设备列表，或提前在树莓派上安装/启用 mDNS。

## 比赛前检查

```bash
cd /home/kiwivk/drone_mission_project_rpi5_ready
./preflight_remote_check.sh
```

## 正式启动比赛

```bash
cd /home/kiwivk/drone_mission_project_rpi5_ready
./remote_start.sh
```

启动后看日志：

```bash
./remote_logs.sh
```

查看状态：

```bash
./remote_status.sh
```

停止任务：

```bash
./remote_stop.sh
```

## 修改端口/相机/深度硬件

配置文件在：

```bash
sudo nano /etc/drone-mission/mission.env
```

默认内容重点：

```bash
FC_PORT=/dev/ttyAMA0
BOTTOM_CAMERA_PORT=/dev/video0
FRONT_CAMERA_PORT=/dev/video2
DEPTH_BACKEND=oakd
```

如果换成 RealSense：

```bash
DEPTH_BACKEND=realsense
```

如果使用普通深度图设备：

```bash
DEPTH_BACKEND=range_grid
DEPTH_CAMERA_PORT=/dev/video4
```

如果使用多向测距串口模块：

```bash
DEPTH_BACKEND=lidar_serial
LIDAR_SERIAL_PORT=/dev/ttyUSB0
```
