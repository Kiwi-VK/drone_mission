#!/usr/bin/env bash
set -euo pipefail
APP_DIR="${APP_DIR:-/home/kiwivk/drone_mission_project_rpi5_ready}"
SERVICE_USER="${SERVICE_USER:-kiwivk}"

cd "$APP_DIR"

sudo apt-get update
sudo apt-get install -y openssh-server v4l-utils python3-venv python3-pip python3-opencv python3-numpy python3-serial python3-pymavlink
sudo systemctl enable --now ssh
sudo usermod -aG dialout,video,i2c "$SERVICE_USER" || true

if [[ ! -d "$APP_DIR/.venv" ]]; then
  python3 -m venv --system-site-packages "$APP_DIR/.venv"
fi
"$APP_DIR/.venv/bin/python" -m pip install --upgrade pip
"$APP_DIR/.venv/bin/python" -m pip install -r "$APP_DIR/requirements.txt"

sudo mkdir -p /etc/drone-mission
if [[ ! -f /etc/drone-mission/mission.env ]]; then
  sudo cp "$APP_DIR/drone-mission.env.example" /etc/drone-mission/mission.env
fi
sudo cp "$APP_DIR/drone-mission-remote.service" /etc/systemd/system/drone-mission.service
sudo systemctl daemon-reload
sudo systemctl disable drone-mission.service

echo "Remote start setup complete. Reboot once: sudo reboot"
echo "After reboot, from your laptop/phone SSH in and run:"
echo "  cd $APP_DIR"
echo "  ./remote_start.sh"
