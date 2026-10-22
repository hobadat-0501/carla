#!/usr/bin/env python3
"""Launch the MATLAB-derived MPC controller in CARLA."""
from carla_firmware.mpc_carla import run


if __name__ == "__main__":
    raise SystemExit(run())
