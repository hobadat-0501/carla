# CARLA 0.9.15 Simulation Firmware

Python client firmware for reproducible vehicle simulation and data capture on Ubuntu. It uses CARLA's Python API; it does not modify or replace the CARLA server binary.

## Ubuntu setup

Use CPython 3.10 x86_64, Ubuntu with glibc 2.27 or newer, and a CARLA 0.9.15 server. PyPI publishes a matching Linux x86_64 client wheel for this Python version. Start CARLA separately, then:

```bash
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python carla_sim.py basic --help
python carla_sim.py realtime --help
```

The server and client must both be 0.9.15. Run one synchronous client at a time. `pad` mode creates an OpenDRIVE test road and controls an ego vehicle. `attach` mode is read-only and observes an existing vehicle without changing the world or ticking it.

## Main commands

```bash
python carla_sim.py basic --case ngang --out ket_qua/ngang_01
python carla_sim.py maneuver --maneuver jturn --speed-kmh 30 --mu-label 1.0 --tire-friction 1.0 --out ket_qua/jturn_01
python carla_sim.py realtime --mode pad --dt 0.01 --duration 60 --out ket_qua/realtime_01
python carla_sim.py realtime --mode attach --role-name hero --duration 0 --out ket_qua/observe_01
python carla_sim.py fit --runs ket_qua/ngang_01 --out ket_qua/model.json
```

Select a new output directory for every run.

## API and telemetry limits

`carla_firmware/api.py` is the stable client-side API. It reports versions, vehicle physics, snapshot state, controls, weather, map data, and wheel steering. Published [CARLA 0.9.15](https://carla.readthedocs.io/en/0.9.15/python_api/) and [0.9.16](https://carla.readthedocs.io/en/0.9.16/python_api/) Python APIs do not document a getter for per-wheel tire load, force, slip, or angular speed; 0.9.16's `show_debug_telemetry()` displays a debug overlay but does not return those values to Python. The compatibility layer preserves those fields as null and exposes state-derived axle slip angles as estimates. It probes for getters so a custom server/client build can supply native fields, but it cannot add telemetry to a stock server.

All output distinguishes measured CARLA state, configured physics, and estimates. `Fx_total_est_N`/`Fy_total_est_N` are mass times body acceleration, not direct tire force. A friction scalar is a simulator setting, not a measured road friction coefficient.

## Output files

Each run writes `states.csv` or `states_realtime.csv`, `metadata.json`, `tham_so_carla.json`, `mo_hinh_truc_tuyen.json`, `collision_events.json`, and `run_status.json`. The CSV keeps the parameter names and units from `THAM_SO_TOAN_BO_CHUONG_TRINH.md`. Check `run_status.json` before using a run; only `duration_complete` means the requested duration or maneuver finished.
