#!/usr/bin/env bash
set -euo pipefail
cd /home/kiwivk/drone_mission_project_rpi5_ready
.venv/bin/python acceptance_test.py
.venv/bin/python mission.py --simulation
FC_PORT=${FC_PORT:-/dev/ttyAMA0} .venv/bin/python test_connection.py --no-depth
