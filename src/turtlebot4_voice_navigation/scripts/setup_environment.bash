#!/usr/bin/env bash
# Sets up Python virtual environment, dependencies, and offline Vosk speech models.
set -euo pipefail

_pkg_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
_ws_root="$(cd -- "$_pkg_dir/../.." && pwd)"
_cache_dir="$_ws_root/.cache_voice"
_venv_dir="$_ws_root/.venv_gesture"

mkdir -p "$_cache_dir"

echo "[*] Setting up Python dependencies for Voice Navigation..."

# Re-use .venv_gesture or create if missing
if [[ ! -d "$_venv_dir" ]]; then
    python3 -m venv --system-site-packages --without-pip "$_venv_dir"
fi

# Ensure pip is installed
if ! "$_venv_dir/bin/python" -m pip --version >/dev/null 2>&1; then
    if [[ -x "$_ws_root/.venv_yolo/bin/python" ]]; then
        "$_ws_root/.venv_yolo/bin/python" -m pip --python "$_venv_dir/bin/python" install pip
    else
        python3 - "$_cache_dir/get-pip.py" <<'PY'
import sys, urllib.request
urllib.request.urlretrieve('https://bootstrap.pypa.io/get-pip.py', sys.argv[1])
PY
        "$_venv_dir/bin/python" "$_cache_dir/get-pip.py"
    fi
fi

echo "[*] Installing vosk and sounddevice..."
"$_venv_dir/bin/python" -m pip install -q "vosk>=0.3.45" "sounddevice>=0.4.6" "pyyaml"

# Download Vosk Vietnamese Model (Small offline model ~40MB)
_vn_model_dir="$_cache_dir/vosk-model-small-vn"
if [[ ! -d "$_vn_model_dir" ]]; then
    echo "[*] Downloading Vosk Vietnamese Speech Model (small-vn ~40MB)..."
    _zip_path="$_cache_dir/vosk-model-small-vn-0.4.zip"
    if [[ ! -f "$_zip_path" ]]; then
        curl -fSL -o "$_zip_path" "https://alphacephei.com/vosk/models/vosk-model-small-vn-0.4.zip" || {
            echo "[!] Warning: Model download failed (network issue). Text-fallback mode remains 100% functional."
        }
    fi
    if [[ -f "$_zip_path" ]]; then
        unzip -q -o "$_zip_path" -d "$_cache_dir"
        mv "$_cache_dir/vosk-model-small-vn-0.4" "$_vn_model_dir" || true
        rm -f "$_zip_path"
        echo "[+] Vosk Vietnamese model installed at: $_vn_model_dir"
    fi
else
    echo "[+] Vosk Vietnamese model already present at: $_vn_model_dir"
fi

echo "[OK] Environment setup complete."
