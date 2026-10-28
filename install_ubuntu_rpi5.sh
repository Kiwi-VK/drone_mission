#!/usr/bin/env bash
set -euo pipefail

APP_DIR="${APP_DIR:-/home/kiwivk/drone_mission_project_rpi5_ready}"
SERVICE_USER="${SERVICE_USER:-kiwivk}"
PYTHON="${PYTHON:-python3}"

if [[ ! -d "$APP_DIR" ]]; then
  echo "APP_DIR not found: $APP_DIR"
  echo "Put this project at /home/kiwivk/drone_mission_project_rpi5_ready or run: APP_DIR=/path/to/project ./install_ubuntu_rpi5.sh"
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  python3 python3-venv python3-pip python3-opencv python3-numpy python3-serial \
  python3-dev build-essential v4l-utils i2c-tools git

sudo usermod -aG dialout,video,i2c "$SERVICE_USER" || true

# Ubuntu on Raspberry Pi: enable UART through config.txt when available.
CONFIG_TXT="/boot/firmware/config.txt"
CMDLINE_TXT="/boot/firmware/cmdline.txt"
if [[ -f "$CONFIG_TXT" ]]; then
  grep -q '^enable_uart=1' "$CONFIG_TXT" || echo 'enable_uart=1' | sudo tee -a "$CONFIG_TXT" >/dev/null
  # Pi 5 usually maps GPIO14/15 primary UART through /dev/serial0 when enabled.
fi
if [[ -f "$CMDLINE_TXT" ]]; then
  sudo sed -i 's/console=serial0,[0-9]\+ //g; s/console=ttyAMA[0-9],[0-9]\+ //g' "$CMDLINE_TXT" || true
fi
sudo systemctl disable --now serial-getty@serial0.service 2>/dev/null || true
sudo systemctl disable --now serial-getty@ttyAMA0.service 2>/dev/null || true

cd "$APP_DIR"
$PYTHON -m venv --system-site-packages .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt

sudo tee /etc/systemd/system/drone-mission.service >/dev/null <<EOF
[Unit]
Description=Drone mission controller on Ubuntu Raspberry Pi 5
After=network-online.target
Wants=network-online.target
ConditionPathExists=/etc/drone-mission/enable-real-flight

[Service]
Type=oneshot
User=$SERVICE_USER
WorkingDirectory=$APP_DIR
Environment=FC_PORT=/dev/serial0
Environment=FC_BAUD=921600
ExecStart=$APP_DIR/.venv/bin/python $APP_DIR/mission.py --execute --confirm-real
StandardOutput=journal
StandardError=journal
TimeoutStartSec=0

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl disable drone-mission.service

echo
 echo "Ubuntu install complete. Reboot first: sudo reboot"
echo "After reboot check:"
echo "  ls -l /dev/serial0 /dev/ttyAMA0 2>/dev/null || true"
echo "  groups $SERVICE_USER"
echo "  $APP_DIR/.venv/bin/python $APP_DIR/test_connection.py --no-depth"
echo
 echo "Real flight service is gated by /etc/drone-mission/enable-real-flight and remains disabled by default."
