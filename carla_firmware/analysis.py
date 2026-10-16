"""Fit a lightweight bicycle model or compare its continuous replay to CARLA CSVs."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import nnls

from .runtime import write_json
from .model import advance, load_parameters


def _read_runs(paths):
    records = []
    metadata = None
    for run_id, path in enumerate(paths):
        path = Path(path)
        if path.is_dir():
            metadata_path = path / "metadata.json"
            if metadata_path.exists():
                metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            path = path / "states.csv"
            if not path.exists():
                path = path.parent / "states_realtime.csv"
        with path.open(newline="", encoding="utf-8") as stream:
            run_rows = list(csv.DictReader(stream))
            for row in run_rows:
                row["_run_id"] = run_id
            records.extend(run_rows)
    return records, metadata


def fit(args):
    rows, metadata = _read_runs(args.runs)
    if len(rows) < 100:
        raise ValueError("At least 100 measured samples are required")
    L = float(args.wheelbase or metadata["wheelbase_m"])
    mass = float(args.mass or metadata["mass_kg"])
    lf, lr = args.lf_fraction * L, (1 - args.lf_fraction) * L
    matrix, target = [], []
    longitudinal_matrix, longitudinal_target = [], []
    previous = None
    for row in rows:
        if previous is not None and row["_run_id"] != previous["_run_id"]:
            previous = row
            continue
        vx, vy, r, delta = (float(row[k]) for k in ("vx_mps", "vy_mps", "r_radps", "delta_equiv_rad"))
        ay = float(row["ay_mps2"])
        longitudinal_matrix.append([float(row["throttle"]), -float(row["brake"]),
                                    -math.tanh(max(vx, 0) / .2), -max(vx, 0)])
        longitudinal_target.append(float(row["ax_mps2"]))
        if previous is not None and vx > 3 and abs(ay) < 12:
            dt = float(row["t_sim_s"]) - float(previous["t_sim_s"])
            if dt > 0:
                rdot = (r - float(previous["r_radps"])) / dt
                af = delta - math.atan2(vy + lf*r, vx)
                ar = -math.atan2(vy - lr*r, vx)
                if max(abs(af), abs(ar)) > .003:
                    matrix.extend(([af, ar, 0.0], [lf*af, -lr*ar, -rdot]))
                    target.extend((ay, 0.0))
        previous = row
    coeff, _, rank, _ = np.linalg.lstsq(np.asarray(matrix), np.asarray(target), rcond=None)
    cf, cr, iz_m = map(float, coeff)
    if rank < 3 or min(cf, cr, iz_m) <= 0:
        raise ValueError("Fit is not identifiable with positive physical parameters; collect stronger lateral excitation")
    long_fit, _ = nnls(np.asarray(longitudinal_matrix), np.asarray(longitudinal_target))
    p = {"wheelbase_m": L, "mass_kg": mass, "low_speed_end_mps": .8,
         "lateral": {"cf_over_mass": cf, "cr_over_mass": cr, "lf_fraction": args.lf_fraction,
                     "iz_over_mass": iz_m},
         "longitudinal": {"traction_gain": float(long_fit[0]), "brake_gain": float(long_fit[1]),
                          "rolling_accel": float(long_fit[2]), "linear_drag": float(long_fit[3])},
         "_sources": {"cf_cr": "least-squares from measured lateral acceleration",
                      "iz_over_mass": "least-squares from yaw equation", "samples": len(matrix)//2,
                      "longitudinal": "nonnegative least-squares from measured ax and applied controls",
                      "lf_fraction": "user assumption"}}
    write_json(args.out, p)
    print(f"Wrote fitted model: {args.out}")


def compare(args):
    p = load_parameters(args.parameters)
    rows, _ = _read_runs(args.runs)
    if len(rows) < 2:
        raise ValueError("At least two samples per run are required")
    metrics = []
    for index, row in enumerate(rows[:-1]):
        following = rows[index+1]
        if following["_run_id"] != row["_run_id"]:
            continue
        dt = float(following["t_sim_s"]) - float(row["t_sim_s"])
        if not 0 < dt <= .1:
            continue
        state = np.array([float(row[k]) for k in ("X_m", "Y_m", "psi_rad", "vx_mps", "vy_mps", "r_radps")])
        throttle, brake, delta = (float(row[k]) for k in ("throttle", "brake", "delta_equiv_rad"))
        # Predictions are a one-step replay, initialized from each measured state.
        prediction = advance(state, dt, throttle, brake, delta, p, max_step=args.max_step)
        actual = np.array([float(following[k]) for k in ("X_m", "Y_m", "psi_rad", "vx_mps", "vy_mps", "r_radps")])
        error = prediction - actual
        metrics.append({"t_sim_s": float(row["t_sim_s"]), "err_X_m": error[0], "err_Y_m": error[1],
                        "err_psi_rad": error[2], "err_vx_mps": error[3], "err_vy_mps": error[4],
                        "err_r_radps": error[5]})
    args.out.mkdir(parents=True, exist_ok=False)
    with (args.out / "metrics.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(metrics[0])); writer.writeheader(); writer.writerows(metrics)
    write_json(args.out / "validation_report.json", {"scored_steps": len(metrics), "max_step_s": args.max_step,
                 "method": "one-step replay initialized from each measured row; not open-loop forecasting"})
    print(f"Wrote {len(metrics)} one-step comparisons to {args.out}")


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="command", required=True)
    fit_p = sub.add_parser("fit"); fit_p.add_argument("--runs", nargs="+", type=Path, required=True)
    fit_p.add_argument("--out", type=Path, required=True); fit_p.add_argument("--wheelbase", type=float)
    fit_p.add_argument("--mass", type=float); fit_p.add_argument("--lf-fraction", type=float, default=.45)
    cmp = sub.add_parser("compare"); cmp.add_argument("--parameters", type=Path, required=True)
    cmp.add_argument("--runs", nargs="+", type=Path, required=True); cmp.add_argument("--out", type=Path, required=True)
    cmp.add_argument("--max-step", type=float, default=.001)
    args = p.parse_args()
    (fit if args.command == "fit" else compare)(args)


if __name__ == "__main__":
    main()
