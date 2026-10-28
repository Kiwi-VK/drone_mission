# Ubuntu on Raspberry Pi 5 使用说明

你的路径：

```bash
/home/kiwivk/drone_mission_project_rpi5_ready
```

可以使用。本工程已经按该路径提供 Ubuntu 安装脚本：

```bash
cd /home/kiwivk/drone_mission_project_rpi5_ready
chmod +x install_ubuntu_rpi5.sh
./install_ubuntu_rpi5.sh
sudo reboot
```

重启后检查串口：

```bash
ls -l /dev/serial0 /dev/ttyAMA0 2>/dev/null || true
groups kiwivk
```

测试飞控和相机：

```bash
/home/kiwivk/drone_mission_project_rpi5_ready/.venv/bin/python \
  /home/kiwivk/drone_mission_project_rpi5_ready/test_connection.py --no-depth
```

如果 `/dev/serial0` 不存在，但 `/dev/ttyAMA0` 存在，可这样测试：

```bash
FC_PORT=/dev/ttyAMA0 /home/kiwivk/drone_mission_project_rpi5_ready/.venv/bin/python \
  /home/kiwivk/drone_mission_project_rpi5_ready/test_connection.py --no-depth
```

真实运行前必须设置：

```bash
export FRONT_CAMERA_PORT=/dev/video2
export DEPTH_BACKEND=oakd       # 或 realsense/range_grid/lidar_serial
```

真实飞行命令：

```bash
cd /home/kiwivk/drone_mission_project_rpi5_ready
.venv/bin/python mission.py --execute --confirm-real
```

服务方式默认不会自动运行。只有创建闸门文件后才允许 systemd 启动：

```bash
sudo mkdir -p /etc/drone-mission
sudo touch /etc/drone-mission/enable-real-flight
sudo systemctl start drone-mission.service
```
