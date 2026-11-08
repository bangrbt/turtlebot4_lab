#!/usr/bin/env python3
"""Standalone Microphone & Audio Input Diagnostic Tool for TurtleBot 4 Voice Navigation.
Tests sounddevice availability, lists audio input devices, records 3 seconds of audio,
and measures RMS volume level with an ASCII meter.

Usage:
    python3 test_microphone.py
"""
import sys
import time
import math

print("=" * 70)
print("   TURTLEBOT 4 - KIỂM TRA PHẦN CỨNG MICROPHONE & ÂM THANH")
print("=" * 70)

try:
    import sounddevice as sd
    import numpy as np
except ImportError as err:
    print(f"\n[!] Thư viện âm thanh chưa sẵn sàng: {err}")
    print("Vui lòng kích hoạt môi trường ảo: source src/turtlebot4_voice_navigation/scripts/voice_env.bash")
    print("Hoặc chạy: bash src/turtlebot4_voice_navigation/scripts/setup_environment.bash")
    sys.exit(1)

print("\n1. Quét các thiết bị âm thanh khả dụng trên máy tính...")
devices = sd.query_devices()
input_devices = []
for idx, dev in enumerate(devices):
    if dev.get('max_input_channels', 0) > 0:
        input_devices.append((idx, dev))
        default_tag = " [MẶC ĐỊNH]" if idx == sd.default.device[0] else ""
        print(f"   [{idx}] {dev['name']} (Channels: {dev['max_input_channels']}){default_tag}")

if not input_devices:
    print("[!] Không tìm thấy thiết bị Microphone nào trên hệ thống.")
    print("    Vui lòng cắm tai nghe/micro hoặc kiểm tra cấu hình âm thanh Ubuntu Settings > Sound.")
    sys.exit(1)

default_idx = sd.default.device[0]
if default_idx is None or default_idx < 0:
    default_idx = input_devices[0][0]

print(f"\n2. Thiết bị được chọn để thử nghiệm: [{default_idx}] {devices[default_idx]['name']}")
sample_rate = 16000
duration_sec = 3.5

print(f"\n3. Chuẩn bị ghi âm trong {duration_sec} giây...")
print("   -> Hãy nói to một câu tiếng Việt: 'Đi đến khu vực A' hoặc 'Đi đến phòng thí nghiệm'...")
for count in [3, 2, 1]:
    print(f"   Bắt đầu sau: {count}...", end="\r", flush=True)
    time.sleep(0.7)
print("   >>> ĐANG GHI ÂM! HÃY NÓI VÀO MICRO NGAY BÂY GIỜ... <<<               ")

recorded_blocks = []
peak_rms = 0.0

def callback(indata, frames, time_info, status):
    global peak_rms
    arr = indata[:, 0].astype(np.float32)
    rms = np.sqrt(np.mean(arr ** 2))
    if rms > peak_rms:
        peak_rms = rms
    recorded_blocks.append(indata.copy())
    # Display live audio meter in terminal
    bars = int(min(30, rms * 150))
    meter = "█" * bars + "░" * (30 - bars)
    db = 20 * math.log10(max(1e-5, rms))
    print(f"\r   Âm lượng: [{meter}] RMS: {rms:.3f} ({db:.1f} dB)", end="", flush=True)

try:
    with sd.InputStream(samplerate=sample_rate, channels=1, device=default_idx, callback=callback):
        sd.sleep(int(duration_sec * 1000))
except Exception as e:
    print(f"\n[!] Lỗi mở microphone stream: {e}")
    sys.exit(1)

print("\n\n4. Kết quả kiểm tra âm thanh:")
print(f"   - Mức tín hiệu đỉnh (Peak RMS): {peak_rms:.4f}")

if peak_rms > 0.02:
    print("   [OK] MICROPHONE HOẠT ĐỘNG TỐT! Tín hiệu giọng nói rõ ràng và đủ âm lượng.")
    print("   Bạn có thể sử dụng microphone để điều khiển robot trong ROS 2.")
elif peak_rms > 0.005:
    print("   [CẢNH BÁO] Microphone có tín hiệu nhưng âm lượng hơi nhỏ.")
    print("   -> Gợi ý: Tăng âm lượng Input Volume trong Ubuntu Settings > Sound lên 100%.")
else:
    print("   [CẢNH BÁO] Tín hiệu rất yếu hoặc micro bị tắt tiếng (Muted).")
    print("   -> Vui lòng kiểm tra công tắc bật micro trên tai nghe/laptop hoặc Ubuntu Sound Settings.")

print("\n" + "=" * 70)
