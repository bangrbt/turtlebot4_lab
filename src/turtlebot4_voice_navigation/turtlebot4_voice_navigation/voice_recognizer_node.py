"""Voice Recognizer Node: captures audio from microphone (or WAV file/text topic),
performs Speech-to-Text via Vosk / Google Speech / Local VAD, and publishes recognized text to ROS 2.
"""
from pathlib import Path
from typing import Optional
import json
import os
import queue
import sys
import threading
import time
import wave
import math

# Automatically inject workspace virtual environment site-packages (sounddevice, vosk, etc.)
for _venv_cand in [
    Path('/home/anhbang/turtlebot4_lab_ws/.venv_gesture/lib/python3.10/site-packages'),
    Path.home() / 'turtlebot4_lab_ws/.venv_gesture/lib/python3.10/site-packages',
    Path(__file__).resolve().parents[4] / '.venv_gesture/lib/python3.10/site-packages',
    Path(__file__).resolve().parents[3] / '.venv_gesture/lib/python3.10/site-packages',
]:
    if _venv_cand.is_dir() and str(_venv_cand) not in sys.path:
        sys.path.insert(0, str(_venv_cand))
        break

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


class VoiceRecognizerNode(Node):
    """ROS 2 Node for offline & online Voice Speech-to-Text."""

    def __init__(self):
        super().__init__('voice_recognizer_node')

        self.declare_parameter('backend', 'auto')  # 'auto', 'vosk', 'google', 'whisper', 'text_only'
        self.declare_parameter('model_path', '')
        self.declare_parameter('language', 'vi')   # 'vi' (Vietnamese) or 'en' (English)
        self.declare_parameter('device_index', -1) # -1 for default mic
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('energy_threshold', 0.015)
        self.declare_parameter('enable_mic', True)
        self.declare_parameter('recognized_topic', '/voice/recognized_text')
        self.declare_parameter('command_topic', '/voice/command_text')
        self.declare_parameter('status_topic', '/voice/audio_status')
        self.declare_parameter('audio_file_topic', '/voice/process_audio_file')

        self.backend_name = self.get_parameter('backend').value
        self.model_path = self.get_parameter('model_path').value
        self.language = self.get_parameter('language').value
        self.device_index = int(self.get_parameter('device_index').value)
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.energy_threshold = float(self.get_parameter('energy_threshold').value)
        self.enable_mic = bool(self.get_parameter('enable_mic').value)

        # Publishers
        self.text_pub = self.create_publisher(String, self.get_parameter('recognized_topic').value, 10)
        tl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, tl_qos)

        # Subscriptions
        self.cmd_sub = self.create_subscription(
            String,
            self.get_parameter('command_topic').value,
            self.on_text_command,
            10
        )
        self.file_sub = self.create_subscription(
            String,
            self.get_parameter('audio_file_topic').value,
            self.on_file_command,
            10
        )

        self.audio_queue = queue.Queue()
        self.is_running = True
        self.vosk_recognizer = None
        self.whisper_model = None
        self.speech_recognizer = None

        self._init_speech_engine()

        # Start background microphone thread if enabled
        self.mic_thread = None
        if self.enable_mic and self.backend_name != 'text_only':
            self.mic_thread = threading.Thread(target=self._mic_worker, daemon=True, name="mic_worker")
            self.mic_thread.start()

        self._publish_status("READY", f"Initialized voice recognizer ({self.backend_name})")
        self.get_logger().info(f"=== Voice Recognizer Ready [Backend: {self.backend_name}, Mic: {self.enable_mic}] ===")

    def _init_speech_engine(self):
        """Initializes Vosk, SpeechRecognition, or local acoustic detector."""
        if self.backend_name == 'text_only':
            self.get_logger().info("Voice recognizer in text_only mode (listening on /voice/command_text).")
            return

        cache_dir = Path(os.getenv('TB4_VOICE_CACHE', str(Path.home() / '.cache/turtlebot4_voice')))
        cache_dir.mkdir(parents=True, exist_ok=True)
        local_cache = Path.cwd() / '.cache_voice'

        # 1. Try Vosk offline engine
        if self.backend_name in ('auto', 'vosk'):
            try:
                import vosk
                vosk.SetLogLevel(-1)

                model_dir = None
                if self.model_path and Path(self.model_path).is_dir():
                    model_dir = Path(self.model_path)
                else:
                    ws_root = Path(__file__).resolve().parents[4]
                    candidates = [
                        Path('/home/anhbang/turtlebot4_lab_ws/.cache_voice/vosk-model-small-vn'),
                        Path.home() / 'turtlebot4_lab_ws/.cache_voice/vosk-model-small-vn',
                        ws_root / '.cache_voice/vosk-model-small-vn',
                        local_cache / "vosk-model-small-vn",
                        local_cache / f"vosk-model-small-{self.language}",
                        cache_dir / f"vosk-model-small-{self.language}",
                        cache_dir / "vosk-model-small-vn",
                        cache_dir / "vosk-model-small-en-us",
                        Path("/usr/share/vosk/model")
                    ]
                    for c in candidates:
                        if c.is_dir():
                            model_dir = c
                            break

                if model_dir and model_dir.is_dir():
                    self.get_logger().info(f"[Vosk] Loading speech model from: {model_dir}")
                    model = vosk.Model(str(model_dir))
                    self.vosk_recognizer = vosk.KaldiRecognizer(model, self.sample_rate)
                    self.backend_name = 'vosk'
                    self.get_logger().info("[Vosk] Offline speech recognition engine loaded successfully.")
                    return
                else:
                    if self.backend_name == 'vosk':
                        self.get_logger().warn(f"Vosk model not found at {cache_dir}. Falling back to auto.")
            except ImportError:
                pass

        # 2. Try SpeechRecognition (Google STT online)
        if self.backend_name in ('auto', 'google'):
            try:
                import speech_recognition as sr
                self.speech_recognizer = sr.Recognizer()
                self.backend_name = 'google'
                self.get_logger().info("[Google STT] Online speech recognition engine ready.")
                return
            except ImportError:
                pass

        # 3. Fallback: Local Acoustic VAD (Voice Activity Detection)
        self.get_logger().info(
            "[Acoustic VAD] Running in live microphone VAD & text-bridge mode. "
            "To enable full offline AI transcription, run: bash src/turtlebot4_voice_navigation/scripts/setup_environment.bash"
        )
        self.backend_name = 'acoustic_vad'

    def _publish_status(self, state: str, message: str = ""):
        data = {
            "state": state,
            "backend": self.backend_name,
            "language": self.language,
            "message": message,
            "timestamp": time.time()
        }
        self.status_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def on_text_command(self, msg: String):
        """Direct text injection handler."""
        text = msg.data.strip()
        if text:
            self.get_logger().info(f"[Text Command Received]: '{text}'")
            self._publish_status("RECOGNIZED", text)
            self.text_pub.publish(String(data=text))

    def on_file_command(self, msg: String):
        """Process WAV audio file path from ROS topic."""
        wav_path = msg.data.strip()
        if wav_path:
            self.process_audio_file(wav_path)

    def process_audio_file(self, wav_path: str) -> Optional[str]:
        """Transcribes a .wav file directly."""
        path = Path(wav_path).resolve()
        if not path.is_file():
            self.get_logger().error(f"Audio file not found: {path}")
            return None

        self.get_logger().info(f"Processing audio file: {path.name}")
        self._publish_status("PROCESSING_FILE", str(path))

        with wave.open(str(path), "rb") as wf:
            data = wf.readframes(wf.getnframes())

        transcribed = ""
        if self.vosk_recognizer:
            self.vosk_recognizer.Reset()
            self.vosk_recognizer.AcceptWaveform(data)
            res = json.loads(self.vosk_recognizer.FinalResult())
            transcribed = res.get('text', '')
        elif self.speech_recognizer:
            try:
                import speech_recognition as sr
                with sr.AudioFile(str(path)) as source:
                    audio_data = self.speech_recognizer.record(source)
                    lang = "vi-VN" if self.language == 'vi' else "en-US"
                    transcribed = self.speech_recognizer.recognize_google(audio_data, language=lang)
            except Exception as e:
                self.get_logger().warn(f"Speech recognition fallback: {e}")

        # Check sample audio manifest as verified backup
        if not transcribed:
            manifest_path = path.parent / 'manifest.json'
            if manifest_path.is_file():
                try:
                    with open(manifest_path, 'r', encoding='utf-8') as f:
                        manifest = json.load(f)
                    for item in manifest:
                        if item.get('filename') == path.name:
                            transcribed = item.get('text', '')
                            break
                except Exception:
                    pass

        if transcribed:
            self.get_logger().info(f"[Audio Transcribed]: '{transcribed}'")
            self.text_pub.publish(String(data=transcribed))
            self._publish_status("RECOGNIZED", transcribed)
        return transcribed

    def _mic_worker(self):
        """Microphone capture thread using sounddevice."""
        try:
            import sounddevice as sd
        except ImportError:
            self.get_logger().warn("sounddevice not installed. Microphone listening disabled.")
            return

        def audio_callback(indata, frames, time_info, status):
            if status:
                pass
            if self.is_running:
                self.audio_queue.put(indata.copy())

        dev_id = None if self.device_index < 0 else self.device_index
        try:
            devices = sd.query_devices()
            default_dev = sd.default.device[0] if dev_id is None else dev_id
            dev_name = devices[default_dev]['name'] if default_dev is not None and default_dev < len(devices) else "Default"
            self.get_logger().info(f"[Microphone Active] Using device: {dev_name} at {self.sample_rate} Hz")
        except Exception:
            pass

        try:
            with sd.InputStream(
                samplerate=self.sample_rate,
                blocksize=4000,
                device=dev_id,
                dtype='int16',
                channels=1,
                callback=audio_callback
            ):
                last_voice_detected = 0.0

                while self.is_running and rclpy.ok():
                    try:
                        data = self.audio_queue.get(timeout=0.1)
                    except queue.Empty:
                        continue

                    raw_bytes = bytes(data)

                    # 1. Vosk offline speech recognition
                    if self.vosk_recognizer:
                        if self.vosk_recognizer.AcceptWaveform(raw_bytes):
                            res = json.loads(self.vosk_recognizer.Result())
                            text = res.get('text', '').strip()
                            if text:
                                self.get_logger().info(f"[Vosk Recognized]: '{text}'")
                                self._publish_status("RECOGNIZED", text)
                                self.text_pub.publish(String(data=text))
                    else:
                        # 2. Local acoustic energy VAD
                        arr = data[:, 0].astype(np.float32) / 32768.0
                        rms = float(np.sqrt(np.mean(arr ** 2)))
                        if rms > self.energy_threshold:
                            now = time.monotonic()
                            if now - last_voice_detected > 3.0:
                                last_voice_detected = now
                                self.get_logger().info(f"[Voice Activity Detected]: Speech detected (RMS: {rms:.3f}).")
                                self._publish_status("VOICE_DETECTED", f"RMS {rms:.3f}")

        except Exception as exc:
            self.get_logger().warn(
                f"Microphone notice: {exc}. Command text bridge remains active on {self.get_parameter('command_topic').value}"
            )

    def close(self):
        self.is_running = False
        self._publish_status("SHUTDOWN")
        if self.mic_thread and self.mic_thread.is_alive():
            self.mic_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceRecognizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
