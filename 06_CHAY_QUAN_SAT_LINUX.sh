#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$(realpath "$0")")"
PYTHON="${PYTHON:-.venv/bin/python}"
exec "$PYTHON" carla_sim.py realtime --mode attach --role-name hero --duration 0 "$@"
