#!/usr/bin/env bash
set -euo pipefail
sudo mkdir -p /etc/drone-mission
sudo touch /etc/drone-mission/enable-real-flight
sudo systemctl start drone-mission.service
