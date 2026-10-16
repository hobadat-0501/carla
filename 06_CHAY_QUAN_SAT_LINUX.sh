#!/usr/bin/env bash
set -u
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ -n "${PYTHON_CARLA:-}" ]]; then
    carla_python="$PYTHON_CARLA"
elif [[ -x .venv_0915/bin/python ]]; then
    carla_python="$PWD/.venv_0915/bin/python"
elif command -v python3.10 >/dev/null 2>&1; then
    carla_python="$(command -v python3.10)"
else
    carla_python="python3"
fi
carla_run_out="runs/doc_quan_sat_$(date +%Y%m%d_%H%M%S_%N)"
echo "Python: $carla_python"
echo "Giu CARLA server 0.9.15 dang mo; dong cac client khac dang tick."
echo "Camera bam xe, chu ky theo thoi gian thuc; xe giu lai sau khi ket thuc."
"$carla_python" -u buoc1_carla.py --case doc --observe --out "$carla_run_out" "$@"
carla_run_code=$?
echo "Trang thai: $carla_run_out/run_status.json"
if [[ "$carla_run_code" -eq 0 ]]; then
    "$carla_python" buoc1_carla.py --analyze "$carla_run_out"
    carla_run_code=$?
fi
exit "$carla_run_code"
