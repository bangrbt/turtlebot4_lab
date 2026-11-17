#!/usr/bin/env bash
set -euo pipefail
_gesture_pkg_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
_gesture_ws="$(cd -- "$_gesture_pkg_dir/../.." && pwd)"
_gesture_venv="$_gesture_ws/.venv_gesture"
python3 -m venv --system-site-packages --without-pip "$_gesture_venv"
if ! "$_gesture_venv/bin/python" -m pip --version >/dev/null 2>&1; then
    if [[ -x "$_gesture_ws/.venv_yolo/bin/python" ]]; then
        "$_gesture_ws/.venv_yolo/bin/python" -m pip --python "$_gesture_venv/bin/python" install pip
    else
        mkdir -p "$_gesture_ws/.cache_gesture"
        python3 - "$_gesture_ws/.cache_gesture/get-pip.py" <<'PY'
import sys,urllib.request
urllib.request.urlretrieve('https://bootstrap.pypa.io/get-pip.py',sys.argv[1])
PY
        "$_gesture_venv/bin/python" "$_gesture_ws/.cache_gesture/get-pip.py"
    fi
fi
"$_gesture_venv/bin/python" -m pip install -r "$_gesture_pkg_dir/requirements.txt"
"$_gesture_venv/bin/python" "$_gesture_pkg_dir/scripts/download_model.py" --cache "$_gesture_ws/.cache_gesture"
echo 'Source scripts/gesture_env.bash in each terminal, then build/launch as documented.'
