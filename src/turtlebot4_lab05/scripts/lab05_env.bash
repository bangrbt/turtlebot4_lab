# Source from the workspace root after lab_env.bash.
if [[ -z "${TB4_LAB_WS:-}" ]]; then
    echo 'Source lab_env.bash first.' >&2
    return 1
fi
export TB4_LAB05_CACHE="$TB4_LAB_WS/.cache_lab05"
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export YOLO_CONFIG_DIR="$TB4_LAB05_CACHE/ultralytics"
export YOLO_AUTOINSTALL=false
if [[ ! -x "$TB4_LAB_WS/.venv_yolo/bin/python" ]]; then
    echo 'Create .venv_yolo first; see README.' >&2
    return 1
fi
source "$TB4_LAB_WS/.venv_yolo/bin/activate"
# colcon-built ROS console scripts use /usr/bin/python3. Expose this venv's
# site-packages to that interpreter as well; binaries stay ABI-compatible 3.10.
export PYTHONPATH="$TB4_LAB_WS/.venv_yolo/lib/python3.10/site-packages:${PYTHONPATH:-}"
