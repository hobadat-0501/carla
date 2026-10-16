#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$(realpath "$0")")"
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python carla_sim.py basic --help
