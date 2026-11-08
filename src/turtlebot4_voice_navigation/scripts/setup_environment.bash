#!/usr/bin/env bash
# Sets up Python virtual environment, dependencies, and offline Vosk speech models for Lab 07.
set -euo pipefail

_pkg_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
_ws_root="$(cd -- "$_pkg_dir/../.." && pwd)"
_cache_dir="$_ws_root/.cache_voice"
_venv_dir="$_ws_root/.venv_gesture"

mkdir -p "$_cache_dir"

echo "========================================================================"
echo "   CÀI ĐẶT MÔI TRƯỜNG NHẬN DẠNG GIỌNG NÓI CHO TURTLEBOT 4 (LAB 07)    "
echo "========================================================================"

# 1. Ensure .venv_gesture has pip
if [[ ! -x "$_venv_dir/bin/pip" ]]; then
    if [[ -d "$_ws_root/.venv_yolo/lib/python3.10/site-packages/pip" ]]; then
        cp -r "$_ws_root/.venv_yolo/lib/python3.10/site-packages/pip"* "$_venv_dir/lib/python3.10/site-packages/"
        cp "$_ws_root/.venv_yolo/bin/pip"* "$_venv_dir/bin/"
        sed -i 's/\.venv_yolo/\.venv_gesture/g' "$_venv_dir/bin/pip"*
    fi
fi

# 2. Install Speech Recognition and Audio Libraries
echo "[*] Cài đặt các thư viện Python: vosk, SpeechRecognition, sounddevice, pyyaml..."
"$_venv_dir/bin/pip" install -q "vosk>=0.3.45" "SpeechRecognition>=3.10.0" "sounddevice>=0.4.6" "pyyaml" || {
    echo "[!] Chú ý: Pip install cần kết nối mạng. Nếu máy đang offline, hệ thống vẫn hoạt động với chế độ Audio VAD và Terminal."
}

# 3. Download and unpack Vosk Vietnamese Speech Model (small-vn ~40MB)
_vn_model_dir="$_cache_dir/vosk-model-small-vn"
if [[ ! -d "$_vn_model_dir" ]]; then
    echo "[*] Đang tải mô hình nhận dạng tiếng Việt Vosk (vosk-model-small-vn ~40MB)..."
    _zip_path="$_cache_dir/vosk-model-small-vn-0.4.zip"
    if [[ ! -f "$_zip_path" ]]; then
        curl -fSL -o "$_zip_path" "https://alphacephei.com/vosk/models/vosk-model-small-vn-0.4.zip" || {
            echo "[!] Lưu ý: Tải model offline chưa thành công do mạng. Node vẫn hỗ trợ nhận diện qua Google STT và nhập lệnh trực tiếp."
        }
    fi
    if [[ -f "$_zip_path" ]]; then
        unzip -q -o "$_zip_path" -d "$_cache_dir"
        mv "$_cache_dir/vosk-model-small-vn-0.4" "$_vn_model_dir" || true
        rm -f "$_zip_path"
        echo "[+] Đã giải nén mô hình tiếng Việt vào: $_vn_model_dir"
    fi
else
    echo "[+] Mô hình tiếng Việt Vosk đã sẵn sàng tại: $_vn_model_dir"
fi

# 4. Generate environment map, SDF world, and sample audio
echo "[*] Khởi tạo bản đồ 5 khu vực và thế giới mô phỏng..."
"$_venv_dir/bin/python" "$_pkg_dir/scripts/generate_environment.py"
"$_venv_dir/bin/python" "$_pkg_dir/scripts/generate_sample_audio.py"

echo "========================================================================"
echo "[OK] Hoàn tất thiết lập môi trường Voice Navigation thành công!"
echo "========================================================================"
