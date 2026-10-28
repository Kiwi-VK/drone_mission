#!/usr/bin/env bash
set -euo pipefail
APP_DIR="${APP_DIR:-/opt/drone_mission_project}"
SERVICE_USER="${SUDO_USER:-${USER}}"
[[ "$(uname -m)" == "aarch64" ]] || { echo 'Need Raspberry Pi OS 64-bit on Raspberry Pi 5'; exit 1; }
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-opencv python3-numpy python3-serial python3-pymavlink v4l-utils
sudo mkdir -p "$APP_DIR"; sudo cp -a . "$APP_DIR/"; sudo chown -R "$SERVICE_USER":"$SERVICE_USER" "$APP_DIR"
sudo usermod -aG dialout,video "$SERVICE_USER"
sudo sed -i 's/console=serial0,[0-9]\+ //g' /boot/firmware/cmdline.txt || true
grep -q '^enable_uart=1' /boot/firmware/config.txt || echo 'enable_uart=1' | sudo tee -a /boot/firmware/config.txt >/dev/null
sudo systemctl disable --now serial-getty@serial0.service 2>/dev/null || true
python3 -m venv --system-site-packages "$APP_DIR/.venv"
"$APP_DIR/.venv/bin/pip" install -r "$APP_DIR/requirements.txt"
sudo cp "$APP_DIR/drone-mission.service" /etc/systemd/system/drone-mission.service
sudo systemctl daemon-reload; sudo systemctl disable drone-mission.service
printf '%s\n' 'Install complete. Reboot, then run: ' "$APP_DIR/.venv/bin/python $APP_DIR/acceptance_test.py" "$APP_DIR/.venv/bin/python $APP_DIR/mission.py --check-config"
