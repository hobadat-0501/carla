"""Export plots and an MP4 from a recorded CARLA states_realtime.csv run."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import imageio.v2 as imageio
import imageio_ffmpeg
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.ticker import MaxNLocator


def export(run_dir: Path, out_dir: Path, fps: int = 30):
    run_dir, out_dir = run_dir.resolve(), out_dir.resolve()
    rows = list(csv.DictReader((run_dir / "states_realtime.csv").open(encoding="utf-8", newline="")))
    if len(rows) < 2:
        raise ValueError("states_realtime.csv must contain at least two samples")
    status = json.loads((run_dir / "run_status.json").read_text(encoding="utf-8"))
    metadata = json.loads((run_dir / "metadata.json").read_text(encoding="utf-8"))
    def col(key):
        return np.array([float(r[key]) if r.get(key) not in ("", "nan", "NaN", None) else np.nan for r in rows])
    t = col("t_s"); x = col("X_m"); y = col("Y_m")
    speed = col("speed_kmh"); steer = col("steer_applied_norm")
    throttle = col("throttle_applied"); brake = col("brake_applied")
    ay = col("ay_mps2"); yaw_rate = col("r_radps")
    phase = np.array([r.get("phase", "") for r in rows])
    valid = np.isfinite(t + x + y + speed)
    t, x, y, speed, steer, throttle, brake, ay, yaw_rate, phase = [a[valid] for a in
        (t, x, y, speed, steer, throttle, brake, ay, yaw_rate, phase)]
    out_dir.mkdir(parents=True, exist_ok=True)
    matplotlib.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
    png = out_dir / "carla_run_results.png"
    mp4 = out_dir / "carla_run_playback.mp4"
    plt.rcParams.update({"figure.facecolor":"white", "axes.facecolor":"white", "axes.edgecolor":"#526070",
                         "axes.labelcolor":"#263342", "text.color":"#17212b", "xtick.color":"#526070",
                         "ytick.color":"#526070", "grid.color":"#dbe2ea", "font.size":10})
    status_text = f"CARLA {metadata['versions']['server']} | {metadata['vehicle']} | {status['simulated_s']:.2f} s | {status['result']}"
    fig, axs = plt.subplots(2, 2, figsize=(14, 8), constrained_layout=True)
    fig.suptitle("Recorded CARLA run — measured vehicle response\n" + status_text, fontsize=14, fontweight="bold")
    ax = axs[0, 0]
    ax.plot(x, y, color="#1769aa", lw=2, label="CARLA vehicle path")
    ax.scatter([x[0]], [y[0]], color="#16834b", s=45, zorder=3, label="Start")
    ax.scatter([x[-1]], [y[-1]], color="#c24135", s=45, marker="s", zorder=3, label="End")
    ax.set(title="Measured trajectory", xlabel="World X (m)", ylabel="World Y (m)"); ax.axis("equal"); ax.legend(loc="best")
    ax = axs[0, 1]
    ax.plot(t, speed, color="#1769aa", lw=1.7)
    ax.set(title="Measured speed", xlabel="Time (s)", ylabel="Speed (km/h)"); ax.yaxis.set_major_locator(MaxNLocator(6))
    ax = axs[1, 0]
    ax.plot(t, steer, color="#7c3aed", lw=1.3, label="Steer normalized")
    ax.plot(t, throttle, color="#16834b", lw=1.3, label="Throttle")
    ax.plot(t, brake, color="#c24135", lw=1.3, label="Brake")
    ax.set(title="Applied CARLA controls", xlabel="Time (s)", ylabel="Normalized command"); ax.legend(loc="best", ncols=3)
    ax = axs[1, 1]
    ax.plot(t, ay, color="#e28a1b", lw=1.3, label="Lateral acceleration")
    ax.plot(t, yaw_rate, color="#1769aa", lw=1.3, label="Yaw rate")
    ax.set(title="Measured lateral response", xlabel="Time (s)", ylabel="m/s² and rad/s"); ax.legend(loc="best")
    for ax in axs.flat:
        ax.grid(True, alpha=.8); ax.margins(x=.03, y=.1)
    fig.savefig(png, dpi=180, facecolor="white")
    plt.close(fig)

    # Animate only measurements from the CARLA CSV; no simulated states are added.
    fig, axs = plt.subplots(1, 2, figsize=(12.8, 7.2), constrained_layout=True)
    fig.suptitle("CARLA recorded-run playback | " + status_text, fontsize=13, fontweight="bold")
    path_ax, speed_ax = axs
    path_ax.plot(x, y, color="#d7dee7", lw=1.1, label="Recorded full path")
    path_line, = path_ax.plot([], [], color="#1769aa", lw=2.2, label="Path to current time")
    car_marker, = path_ax.plot([], [], "o", color="#c24135", ms=8, label="Vehicle")
    path_ax.set(title="CARLA world trajectory", xlabel="World X (m)", ylabel="World Y (m)")
    path_ax.axis("equal"); path_ax.legend(loc="best"); path_ax.grid(True, alpha=.8)
    speed_ax.plot(t, speed, color="#d7dee7", lw=1.1)
    speed_line, = speed_ax.plot([], [], color="#1769aa", lw=2)
    speed_marker, = speed_ax.plot([], [], "o", color="#c24135", ms=7)
    time_cursor = speed_ax.axvline(t[0], color="#64748b", ls="--", lw=1)
    speed_ax.set(title="Measured speed", xlabel="Time (s)", ylabel="Speed (km/h)")
    speed_ax.set_xlim(float(t[0]), float(t[-1])); speed_ax.set_ylim(0, max(5, float(np.nanmax(speed))*1.15)); speed_ax.grid(True, alpha=.8)
    phase_label = fig.text(.5, .02, "", ha="center", color="#526070")
    frame_count = max(2, int(round((t[-1] - t[0]) * fps)) + 1)
    frame_idx = np.linspace(0, len(t)-1, frame_count).round().astype(int)
    def update(frame):
        i = int(frame_idx[frame])
        path_line.set_data(x[:i+1], y[:i+1]); car_marker.set_data([x[i]], [y[i]])
        speed_line.set_data(t[:i+1], speed[:i+1]); speed_marker.set_data([t[i]], [speed[i]])
        time_cursor.set_xdata([t[i], t[i]])
        phase_label.set_text(f"t = {t[i]:.2f} s    |    phase = {phase[i]}    |    speed = {speed[i]:.1f} km/h")
        return path_line, car_marker, speed_line, speed_marker, time_cursor, phase_label
    animation = FuncAnimation(fig, update, frames=frame_count, interval=1000/fps, blit=False)
    from matplotlib.animation import FFMpegWriter
    animation.save(mp4, writer=FFMpegWriter(fps=fps, codec="libx264", extra_args=["-pix_fmt", "yuv420p"]), dpi=100)
    plt.close(fig)
    return png, mp4, len(t)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="CARLA run folder with states_realtime.csv and metadata.json")
    parser.add_argument("--out", type=Path, help="output directory; defaults to run_dir")
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args(argv)
    png, mp4, n = export(args.run_dir, args.out or args.run_dir, args.fps)
    print(f"CARLA samples: {n}")
    print(f"Image: {png}")
    print(f"Video: {mp4}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
