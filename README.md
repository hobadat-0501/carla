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

## MATLAB kinematic MPC in CARLA

`carla_mpc.py` runs a Python port of `matlab/mpc/kin_mpc.m` on a live CARLA vehicle. It uses the same MPC sample time (0.1 s), prediction step (0.2 s), horizon (8), cost weights, acceleration limits, steering limits, and steering slew limits. The kinematic geometry is read from the spawned CARLA vehicle's physics. The MATLAB `wind07`, `wind10`, `sine10`, and `sine15` reference paths are available. The runner generates its world from the project's RoadRunner OpenDRIVE map in `map/scene.xodr`; it does not load Town maps.

Start CARLA 0.9.15, activate the environment described above, then run from the project directory:

```bash
python carla_mpc.py --scenario wind10 --duration 60 --plot --out ket_qua/mpc_wind10_01
```

By default, the vehicle spawns on OpenDRIVE road 1, lane -1, at `s=100 m`. Change the custom map or spawn location with `--map-file`, `--road-id`, `--lane-id`, and `--spawn-s`.
The CARLA MPC uses a lower steering slew penalty than the MATLAB ideal-plant run so it can respond to the vehicle dynamics. Adjust it with `--steer-slew-weight` if needed (default `50`; the prior value was `500`). The run ends when the reference path is complete or `--duration` is reached.

For the sine path:

```bash
python carla_mpc.py --scenario sine10 --duration 60 --plot --out ket_qua/mpc_sine10_01
```

To show live plots while the CARLA vehicle is driving (requires a graphical desktop), add `--plot`:

```bash
python carla_mpc.py --scenario wind10 --duration 60 --plot --out ket_qua/mpc_wind10_live_01
```

The live window shows CARLA vehicle position against the reference, measured/reference speed, and cross-track error. At the end it saves `mpc_live_plot.png` beside `states_mpc.csv`.

The runner loads the selected map, spawns a Tesla Model 3, and writes `states_mpc.csv` plus `run_status.txt`. Use a new output folder for every run. Unlike MATLAB's plant simulation, CARLA supplies the actual vehicle dynamics; the MATLAB reference tracking logic is preserved, while the longitudinal acceleration request is converted to CARLA throttle/brake using measured speed feedback. This is an initial live-simulation port and should be tuned for the selected vehicle/map before interpreting tracking errors.

Select a new output directory for every run.

## Visible dynamics tests

`scenario` defaults to the built-in CARLA `Town03` city map (or accepts another built-in map with `--map`) and uses the selected vehicle spawn point. For highway and large-road testing, use `Town06`. It reloads that world at each launch, which clears actors and old debug drawings. The CARLA window shows a separated live HUD with phase, speed, body velocities, accelerations, yaw rate, steering, throttle, and brake; a short trajectory trail is drawn behind the vehicle.

Run the complete sequence in one CARLA world and one CSV. Its `phase` column and HUD label move through straight, speed step, coastdown, recovery, steering ramp, J-turn, double pulse, sine-with-dwell, constant radius, and braking:

```bash
python carla_sim.py scenario --scenario suite --map Town06 --spawn-index 0 --speed-kmh 25 --step-speed-kmh 35 --duration 60 --out ket_qua/town06_suite_01
```

Individual cases can also be run separately on the same map:

```bash
python carla_sim.py scenario --scenario straight --speed-kmh 25 --out ket_qua/straight_01
python carla_sim.py scenario --scenario speed_step --speed-kmh 25 --step-speed-kmh 35 --out ket_qua/speed_step_01
python carla_sim.py scenario --scenario coastdown --speed-kmh 25 --out ket_qua/coastdown_01
python carla_sim.py scenario --scenario steer_ramp --speed-kmh 25 --delta-deg 2 --out ket_qua/steer_ramp_01
python carla_sim.py scenario --scenario jturn --speed-kmh 25 --delta-deg 2 --out ket_qua/jturn_01
python carla_sim.py scenario --scenario double_pulse --speed-kmh 25 --delta-deg 2 --out ket_qua/double_pulse_01
python carla_sim.py scenario --scenario sine_dwell --speed-kmh 25 --delta-deg 2 --out ket_qua/sine_dwell_01
python carla_sim.py scenario --scenario constant_radius --speed-kmh 25 --radius-m 60 --out ket_qua/radius_01
python carla_sim.py scenario --scenario braking --speed-kmh 25 --brake-pulse 1 --out ket_qua/braking_01
```

The suite contains ten phases: straight, speed step, coastdown, recovery, steering ramp, J-turn, double pulse, sine-with-dwell, constant radius, and braking. They run sequentially in one loaded map and one CSV. `--duration` is the safety time limit; most individual scenarios finish after `--maneuver-duration` once warmup is complete. `--repeat-s` repeats the maneuver after a short straight segment. These are simulation experiments, not certification tests.

## User-defined drive script

Edit `kich_ban_nguoi_dung.json`. Every item in `steps` specifies a target speed (`speed_kmh`), signed front-wheel steering angle in degrees (`steer_deg`), and duration (`duration_s`). The runner tracks speed with throttle/brake control and applies the requested steering angle for that step. Each sample is recorded in `states_realtime.csv`, including requested targets and measured vehicle state; the exact script and vehicle parameters are also saved in the output folder.

```bash
python carla_sim.py custom --script kich_ban_nguoi_dung.json --out ket_qua/kich_ban_01
```

To enter steps in a desktop window instead of editing JSON, run `python carla_gui.py` from the CARLA Python environment. Add/edit/delete steps in the table and click **Chạy kịch bản**. This control panel is a separate desktop window alongside CARLA; install Ubuntu's `python3-tk` package if Tkinter is missing.

## API and telemetry limits

`carla_firmware/api.py` is the stable client-side API. It reports versions, vehicle physics, snapshot state, controls, weather, map data, and wheel steering. Published [CARLA 0.9.15](https://carla.readthedocs.io/en/0.9.15/python_api/) and [0.9.16](https://carla.readthedocs.io/en/0.9.16/python_api/) Python APIs do not document a getter for per-wheel tire load, force, slip, or angular speed; 0.9.16's `show_debug_telemetry()` displays a debug overlay but does not return those values to Python. The compatibility layer preserves those fields as null and exposes state-derived axle slip angles as estimates. It probes for getters so a custom server/client build can supply native fields, but it cannot add telemetry to a stock server.

All output distinguishes measured CARLA state, configured physics, and estimates. `Fx_total_est_N`/`Fy_total_est_N` are mass times body acceleration, not direct tire force. A friction scalar is a simulator setting, not a measured road friction coefficient.

## Output files

Each run writes `states.csv` or `states_realtime.csv`, `metadata.json`, `tham_so_carla.json`, `mo_hinh_truc_tuyen.json`, `collision_events.json`, and `run_status.json`. The CSV keeps the parameter names and units from `THAM_SO_TOAN_BO_CHUONG_TRINH.md`. Check `run_status.json` before using a run; only `duration_complete` means the requested duration or maneuver finished.
