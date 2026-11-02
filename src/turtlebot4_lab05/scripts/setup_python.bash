#!/usr/bin/env bash
set -euo pipefail
if [[ -z "${TB4_LAB_WS:-}" ]]; then
    echo 'Source workspace lab_env.bash first.' >&2
    exit 1
fi
pkg_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="$TB4_LAB_WS/.venv_yolo"
python3 -m venv --system-site-packages --without-pip "$venv_dir"
if ! "$venv_dir/bin/python" -m pip --version >/dev/null 2>&1; then
    mkdir -p "$TB4_LAB_WS/.cache_lab05"
    bootstrap_file="$TB4_LAB_WS/.cache_lab05/get-pip.py"
    python3 - "$bootstrap_file" <<'PY'
import sys, urllib.request
urllib.request.urlretrieve('https://bootstrap.pypa.io/get-pip.py', sys.argv[1])
PY
    "$venv_dir/bin/python" "$bootstrap_file"
fi
"$venv_dir/bin/python" -m pip install --no-cache-dir torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu121
"$venv_dir/bin/python" -m pip install --no-cache-dir -r "$pkg_dir/requirements-yolo.txt"
echo 'Done. Source scripts/lab05_env.bash after workspace lab_env.bash in each terminal.'
