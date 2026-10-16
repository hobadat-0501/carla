#!/usr/bin/env python3
"""Desktop control panel for user-authored CARLA drive sequences (Ubuntu/Tk)."""
from __future__ import annotations

import json
import csv
import math
import queue
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


ROOT = Path(__file__).resolve().parent


class CarlaControlPanel:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("CARLA 0.9.15 - Kịch bản lái xe")
        root.geometry("940x720")
        self.process = None
        self.messages: queue.Queue[str] = queue.Queue()
        self.steps: list[dict] = []
        self.output_csv: Path | None = None
        self.last_plot_update = 0.0
        self.csv_mtime = None

        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True)
        control_tab = ttk.Frame(notebook)
        plot_tab = ttk.Frame(notebook)
        notebook.add(control_tab, text="Điều khiển")
        notebook.add(plot_tab, text="Telemetry và biểu đồ")
        self.notebook = notebook

        settings = ttk.LabelFrame(control_tab, text="Thiết lập mô phỏng", padding=8)
        settings.pack(fill="x", padx=10, pady=(10, 5))
        self.map_var = tk.StringVar(value="Town06")
        self.spawn_var = tk.StringVar(value="0")
        self.vehicle_var = tk.StringVar(value="vehicle.tesla.model3")
        self.host_var = tk.StringVar(value="127.0.0.1")
        self.port_var = tk.StringVar(value="2000")
        for col, (label, var, width) in enumerate((
            ("Map", self.map_var, 14), ("Spawn index", self.spawn_var, 8),
            ("Vehicle blueprint", self.vehicle_var, 26), ("Host", self.host_var, 14),
            ("Port", self.port_var, 8),
        )):
            ttk.Label(settings, text=label).grid(row=0, column=col, sticky="w", padx=4)
            ttk.Entry(settings, textvariable=var, width=width).grid(row=1, column=col, sticky="ew", padx=4)

        editor = ttk.LabelFrame(control_tab, text="Bước điều khiển", padding=8)
        editor.pack(fill="x", padx=10, pady=5)
        self.name_var = tk.StringVar()
        self.speed_var = tk.StringVar(value="20")
        self.steer_var = tk.StringVar(value="0")
        self.duration_var = tk.StringVar(value="10")
        for col, (label, var) in enumerate((("Tên bước", self.name_var), ("Tốc độ (km/h)", self.speed_var),
                                            ("Góc lái (độ)", self.steer_var), ("Thời gian (s)", self.duration_var))):
            ttk.Label(editor, text=label).grid(row=0, column=col, sticky="w", padx=4)
            ttk.Entry(editor, textvariable=var, width=19).grid(row=1, column=col, sticky="ew", padx=4)
        ttk.Button(editor, text="Thêm bước", command=self.add_step).grid(row=1, column=4, padx=4)
        ttk.Button(editor, text="Cập nhật bước chọn", command=self.update_step).grid(row=1, column=5, padx=4)
        ttk.Button(editor, text="Xóa bước chọn", command=self.delete_step).grid(row=1, column=6, padx=4)

        table_frame = ttk.Frame(control_tab)
        table_frame.pack(fill="both", expand=True, padx=10, pady=5)
        self.table = ttk.Treeview(table_frame, columns=("name", "speed", "steer", "duration"), show="headings", height=8)
        for col, title, width in (("name", "Tên bước", 300), ("speed", "Tốc độ km/h", 150),
                                  ("steer", "Góc lái độ", 150), ("duration", "Thời gian s", 150)):
            self.table.heading(col, text=title)
            self.table.column(col, width=width, anchor="center" if col != "name" else "w")
        self.table.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(table_frame, orient="vertical", command=self.table.yview)
        scrollbar.pack(side="right", fill="y")
        self.table.configure(yscrollcommand=scrollbar.set)
        self.table.bind("<<TreeviewSelect>>", self.select_step)

        actions = ttk.Frame(control_tab)
        actions.pack(fill="x", padx=10, pady=5)
        self.run_button = ttk.Button(actions, text="Chạy kịch bản", command=self.start_run)
        self.run_button.pack(side="left")
        self.stop_button = ttk.Button(actions, text="Dừng", command=self.stop_run, state="disabled")
        self.stop_button.pack(side="left", padx=6)
        ttk.Label(actions, text="Kết quả lưu dưới ket_qua/gui_<thời_gian>").pack(side="left", padx=12)

        output_frame = ttk.LabelFrame(control_tab, text="Tiến trình CARLA", padding=6)
        output_frame.pack(fill="both", expand=True, padx=10, pady=(5, 10))
        self.output = tk.Text(output_frame, height=12, wrap="word", state="disabled")
        self.output.pack(side="left", fill="both", expand=True)
        out_scroll = ttk.Scrollbar(output_frame, orient="vertical", command=self.output.yview)
        out_scroll.pack(side="right", fill="y")
        self.output.configure(yscrollcommand=out_scroll.set)

        self.live_values = ttk.Label(plot_tab, text="Chưa có dữ liệu telemetry", padding=8)
        self.live_values.pack(fill="x")
        self.figure = Figure(figsize=(12, 8), dpi=90, constrained_layout=True)
        self.axes = self.figure.subplots(4, 2).flatten()
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_tab)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.init_plot_axes()
        self.load_example()
        root.after(100, self.poll_process)

    def init_plot_axes(self):
        titles = ("Quỹ đạo XY", "Vận tốc dọc", "Vận tốc góc yaw", "Góc lái bánh trước",
                  "Hướng xe", "Gia tốc dọc", "Gia tốc ngang", "Điều khiển ga/phanh")
        ylabels = ("Y (m)", "km/h", "rad/s", "deg", "deg", "m/s²", "m/s²", "Mức điều khiển")
        for axis, title, ylabel in zip(self.axes, titles, ylabels):
            axis.set_title(title)
            axis.set_ylabel(ylabel)
            axis.grid(True, alpha=.3)
        self.axes[0].set_xlabel("X (m)")
        for axis in self.axes[1:]:
            axis.set_xlabel("Thời gian mô phỏng (s)")
        self.canvas.draw_idle()

    def update_plot(self):
        if not self.output_csv or not self.output_csv.exists():
            return
        try:
            modified = self.output_csv.stat().st_mtime_ns
            if modified == self.csv_mtime:
                return
            with self.output_csv.open("r", newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            if not rows:
                return
            self.csv_mtime = modified
            def values(key):
                result = []
                for row in rows:
                    try:
                        value = float(row.get(key, "nan"))
                    except (TypeError, ValueError):
                        value = float("nan")
                    result.append(value)
                return result

            t = values("t_s")
            x, y = values("X_m"), values("Y_m")
            vx = [value * 3.6 for value in values("vx_mps")]
            vx_set = values("target_speed_kmh")
            steer, steer_set = values("steer_measured_deg"), values("target_steer_deg")
            r = values("r_radps")
            heading = [math.degrees(value) for value in values("psi_rad")]
            ax, ay = values("ax_mps2"), values("ay_mps2")
            throttle, brake = values("throttle_applied"), values("brake_applied")
            for axis, title in zip(self.axes, ("Quỹ đạo XY", "Vận tốc dọc", "Vận tốc góc yaw",
                                               "Góc lái bánh trước", "Hướng xe", "Gia tốc dọc",
                                               "Gia tốc ngang", "Điều khiển ga/phanh")):
                axis.clear()
                axis.set_title(title)
                axis.grid(True, alpha=.3)
            self.axes[0].plot(x, y, color="tab:blue", label="Thực chạy")
            self.axes[0].set_xlabel("X (m)"); self.axes[0].set_ylabel("Y (m)")
            self.axes[0].axis("equal"); self.axes[0].legend(loc="best")

            self.axes[1].plot(t, vx, color="tab:blue", label="Thực chạy Vx")
            self.axes[1].step(t, vx_set, where="post", color="tab:red", linestyle="--", label="Đặt tốc độ")
            self.axes[1].set_ylabel("km/h"); self.axes[1].legend(loc="best")
            self.axes[2].plot(t, r, color="tab:blue", label="Thực chạy")
            self.axes[2].set_ylabel("rad/s"); self.axes[2].legend(loc="best")
            self.axes[3].plot(t, steer, color="tab:blue", label="Thực chạy")
            self.axes[3].step(t, steer_set, where="post", color="tab:red", linestyle="--", label="Đặt góc lái")
            self.axes[3].set_ylabel("deg"); self.axes[3].legend(loc="best")
            self.axes[4].plot(t, heading, color="tab:blue", label="Thực chạy")
            self.axes[4].set_ylabel("deg"); self.axes[4].legend(loc="best")
            self.axes[5].plot(t, ax, color="tab:blue", label="Thực chạy")
            self.axes[5].set_ylabel("m/s²"); self.axes[5].legend(loc="best")
            self.axes[6].plot(t, ay, color="tab:blue", label="Thực chạy")
            self.axes[6].set_ylabel("m/s²"); self.axes[6].legend(loc="best")
            self.axes[7].plot(t, throttle, label="Ga")
            self.axes[7].plot(t, brake, label="Phanh")
            self.axes[7].set_ylabel("0–1"); self.axes[7].legend(loc="best")
            for axis in self.axes[1:]:
                axis.set_xlabel("Thời gian (s)")
            self.canvas.draw_idle()
            last = rows[-1]
            self.live_values.configure(text=(
                f"t={last.get('t_s','')} s | X={last.get('X_m','')} m, Y={last.get('Y_m','')} m | "
                f"Vx={float(last.get('vx_mps', 0))*3.6:.2f} km/h | r={float(last.get('r_radps', 0)):.3f} rad/s | "
                f"góc lái={float(last.get('steer_measured_deg', 0)):.2f}° | "
                f"hướng={math.degrees(float(last.get('psi_rad', 0))):.2f}° | "
                f"ax={float(last.get('ax_mps2', 0)):.2f}, ay={float(last.get('ay_mps2', 0)):.2f} m/s²"))
        except (OSError, csv.Error, ValueError, TypeError):
            return

    def load_example(self):
        path = ROOT / "kich_ban_nguoi_dung.json"
        if path.exists():
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                self.map_var.set(data.get("map", "Town06"))
                self.spawn_var.set(str(data.get("spawn_index", 0)))
                self.vehicle_var.set(data.get("vehicle", "vehicle.tesla.model3"))
                for step in data.get("steps", []):
                    self.steps.append({"name": step.get("name", f"step_{len(self.steps)+1:02d}"),
                                       "speed_kmh": float(step["speed_kmh"]),
                                       "steer_deg": float(step["steer_deg"]),
                                       "duration_s": float(step["duration_s"])})
                self.refresh_table()
            except (OSError, ValueError, KeyError, json.JSONDecodeError):
                pass

    def step_from_form(self):
        try:
            item = {"name": self.name_var.get().strip() or f"step_{len(self.steps)+1:02d}",
                    "speed_kmh": float(self.speed_var.get()), "steer_deg": float(self.steer_var.get()),
                    "duration_s": float(self.duration_var.get())}
        except ValueError:
            raise ValueError("Tốc độ, góc lái và thời gian phải là số.")
        if not 0 <= item["speed_kmh"] <= 100:
            raise ValueError("Tốc độ phải trong khoảng 0–100 km/h.")
        if not -30 <= item["steer_deg"] <= 30:
            raise ValueError("Góc lái phải trong khoảng -30 đến 30 độ.")
        if item["duration_s"] <= 0:
            raise ValueError("Thời gian bước phải lớn hơn 0.")
        return item

    def add_step(self):
        try:
            self.steps.append(self.step_from_form())
            self.refresh_table()
        except ValueError as exc:
            messagebox.showerror("Thông số chưa hợp lệ", str(exc))

    def update_step(self):
        selected = self.table.selection()
        if not selected:
            messagebox.showinfo("Chọn bước", "Hãy chọn một bước trong bảng trước.")
            return
        try:
            self.steps[int(selected[0])] = self.step_from_form()
            self.refresh_table()
        except ValueError as exc:
            messagebox.showerror("Thông số chưa hợp lệ", str(exc))

    def delete_step(self):
        selected = self.table.selection()
        if selected:
            del self.steps[int(selected[0])]
            self.refresh_table()

    def select_step(self, _event=None):
        selected = self.table.selection()
        if selected:
            step = self.steps[int(selected[0])]
            self.name_var.set(step["name"])
            self.speed_var.set(str(step["speed_kmh"]))
            self.steer_var.set(str(step["steer_deg"]))
            self.duration_var.set(str(step["duration_s"]))

    def refresh_table(self):
        self.table.delete(*self.table.get_children())
        for index, step in enumerate(self.steps):
            self.table.insert("", "end", iid=str(index), values=(step["name"], step["speed_kmh"],
                               step["steer_deg"], step["duration_s"]))

    def append_output(self, line):
        self.output.configure(state="normal")
        self.output.insert("end", line)
        self.output.see("end")
        self.output.configure(state="disabled")

    def start_run(self):
        if self.process and self.process.poll() is None:
            return
        if not self.steps:
            messagebox.showerror("Chưa có kịch bản", "Thêm ít nhất một bước điều khiển.")
            return
        try:
            spawn = int(self.spawn_var.get())
            port = int(self.port_var.get())
            if spawn < 0 or not 1 <= port <= 65535:
                raise ValueError
        except ValueError:
            messagebox.showerror("Thông số chưa hợp lệ", "Spawn index hoặc port không hợp lệ.")
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        out = ROOT / "ket_qua" / f"gui_{stamp}"
        script_path = ROOT / "ket_qua" / f"kich_ban_gui_{stamp}.json"
        script_path.parent.mkdir(parents=True, exist_ok=True)
        script = {"map": self.map_var.get().strip() or "Town06", "spawn_index": spawn,
                  "vehicle": self.vehicle_var.get().strip() or "vehicle.tesla.model3", "steps": self.steps}
        script_path.write_text(json.dumps(script, ensure_ascii=False, indent=2), encoding="utf-8")
        command = [sys.executable, str(ROOT / "carla_sim.py"), "custom", "--script", str(script_path),
                   "--host", self.host_var.get().strip() or "127.0.0.1", "--port", str(port), "--out", str(out)]
        self.append_output("\n$ " + " ".join(command) + "\n")
        self.output_csv = out / "states_realtime.csv"
        self.csv_mtime = None
        try:
            self.process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT, text=True, bufsize=1)
        except OSError as exc:
            messagebox.showerror("Không chạy được", str(exc))
            return
        threading.Thread(target=self.read_process, args=(self.process,), daemon=True).start()
        self.run_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.append_output(f"Script: {script_path}\nOutput: {out}\n")

    def read_process(self, process):
        if process.stdout:
            for line in process.stdout:
                self.messages.put(line)

    def poll_process(self):
        while True:
            try:
                self.append_output(self.messages.get_nowait())
            except queue.Empty:
                break
        if self.process and self.process.poll() is not None:
            self.run_button.configure(state="normal")
            self.stop_button.configure(state="disabled")
            self.append_output(f"CARLA runner kết thúc với mã {self.process.returncode}.\n")
            self.process = None
        if time.monotonic() - self.last_plot_update >= 1.0:
            self.update_plot()
            self.last_plot_update = time.monotonic()
        self.root.after(100, self.poll_process)

    def stop_run(self):
        if self.process and self.process.poll() is None:
            self.process.send_signal(2)
            self.append_output("Đã gửi yêu cầu dừng an toàn (Ctrl+C).\n")


def main():
    root = tk.Tk()
    CarlaControlPanel(root)
    root.mainloop()


if __name__ == "__main__":
    main()
