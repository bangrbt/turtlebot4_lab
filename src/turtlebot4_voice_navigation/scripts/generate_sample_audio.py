#!/usr/bin/env python3
"""Generates test WAV audio files and manifest for TurtleBot 4 Voice Navigation (5 zones).
"""
from pathlib import Path
import math
import struct
import wave
import json


def make_tone_wav(filename: Path, duration_sec: float = 2.2, sample_rate: int = 16000, freq: float = 440.0):
    num_samples = int(duration_sec * sample_rate)
    with wave.open(str(filename), 'wb') as wf:
        wf.setnchannels(1)        # mono
        wf.setsampwidth(2)        # 16-bit
        wf.setframerate(sample_rate)
        frames = bytearray()
        for i in range(num_samples):
            t = float(i) / sample_rate
            # Harmonic voice-like waveform with frequency modulation
            f_inst = freq + 15.0 * math.sin(2.0 * math.pi * 3.0 * t)
            val = 0.6 * math.sin(2.0 * math.pi * f_inst * t) + 0.3 * math.sin(4.0 * math.pi * f_inst * t)
            # Envelope (fade in / out)
            env = min(1.0, t * 10.0) * min(1.0, (duration_sec - t) * 10.0)
            sample = int(val * env * 24000.0)
            frames.extend(struct.pack('<h', max(-32768, min(32767, sample))))
        wf.writeframes(frames)


def main():
    pkg_dir = Path(__file__).resolve().parent.parent
    audio_dir = pkg_dir / 'sample_audio'
    audio_dir.mkdir(parents=True, exist_ok=True)

    samples = [
        ("01_di_den_khu_vuc_a.wav", "Đi đến khu vực A", 450.0),
        ("02_di_tu_a_den_b.wav", "Di chuyển từ khu vực A đến khu vực B", 400.0),
        ("03_di_den_phong_thi_nghiem.wav", "Đi đến phòng thí nghiệm", 480.0),
        ("04_di_den_thu_vien.wav", "Đi đến thư viện", 430.0),
        ("05_multi_waypoints.wav", "Đi từ phòng học qua hành lang rồi đến phòng thí nghiệm", 420.0),
        ("06_khu_vuc_khong_ton_tai.wav", "Đi đến phòng tổng thống", 380.0),
    ]

    manifest = []
    for fname, text, f in samples:
        path = audio_dir / fname
        make_tone_wav(path, duration_sec=2.2, sample_rate=16000, freq=f)
        manifest.append({
            "filename": fname,
            "text": text,
            "sample_rate": 16000,
            "channels": 1,
            "format": "PCM 16-bit"
        })
        print(f"[+] Created {path} -> '{text}'")

    with open(audio_dir / 'manifest.json', 'w', encoding='utf-8') as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    print(f"[+] Saved {audio_dir / 'manifest.json'}")


if __name__ == '__main__':
    main()
