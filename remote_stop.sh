#!/usr/bin/env bash
set -euo pipefail
sudo systemctl stop drone-mission.service || true
