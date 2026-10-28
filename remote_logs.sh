#!/usr/bin/env bash
set -euo pipefail
journalctl -u drone-mission.service -f -n 80
