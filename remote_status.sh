#!/usr/bin/env bash
set -euo pipefail
systemctl status drone-mission.service --no-pager || true
