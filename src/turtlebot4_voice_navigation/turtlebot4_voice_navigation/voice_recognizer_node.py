"""Voice Recognizer Node: captures audio from microphone (or WAV file/text topic),
performs Speech-to-Text via Vosk or Whisper, and publishes recognized text to ROS 2.
"""
from pathlib import Path
from typing import Optional
import json
import os
import queue
import threading
import time
import wave
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


class VoiceRecognizerNode(Node):
    """ROS 2 Node for offline Voice Speech-to-Text."""

    def __init__(self):
        super().__init__('voice_recognizer_node')

        self.declare_parameter('backend', 'vosk')  # 'vosk', 'whisper', or 'text_only'
        self.declare_parameter('model_path', '')
        self.declare_parameter('language', 'vi')   # 'vi' (Vietnamese) or 'en' (English)
        self.declare_parameter('device_index', -1) # -1 for default mic
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('energy_threshold', 800)
        self.declare_parameter('enable_mic', True)
        self.declare_parameter('recognized_topic', '/voice/recognized_text')
        self.declare_parameter('command_topic', '/voice/command_text')
        self.declare_parameter('status_topic', '/voice/audio_status')

        self.backend_name = self.get_parameter('backend').value
        self.model_path = self.get_parameter('model_path').value
        self.language = self.get_parameter('language').value
        self.device_index = self.get_parameter('device_index').value
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.energy_threshold = int(self.get_parameter('energy_threshold').value)
        self.enable_mic = bool(self.get_parameter('enable_mic').value)

        # Publishers
        self.text_pub = self.create_publisher(String, self.get_parameter('recognized_topic').value, 10)
        tl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, tl_qos)

        # Subscription for text/CLI fallback injection
        self.cmd_sub = self.create_subscription(
            String,
            self.get_parameter('command_topic').value,
            self.on_text_command,
            10
        )

        self.audio_queue = queue.Queue()
        self.is_running = True
        self.vosk_recognizer = None
        self.whisper_model = None

        self._init_speech_engine()

        # Start background microphone thread if enabled
        self.mic_thread = None
        if self.enable_mic and self.backend_name != 'text_only':
            self.mic_thread = threading.Thread(target=self._mic_worker, daemon=True, name="mic_worker")
            self.mic_thread.start()

        self._publish_status("READY", f"Initialized voice recognizer ({self.backend_name})")
        self.get_logger().info(f"Voice Recognizer initialized. Backend: {self.backend_name}, Mic: {self.enable_mic}")

    def _init_speech_engine(self):
        """Initializes Vosk or Whisper speech recognition engine."""
        if self.backend_name == 'text_only':
            self.get_logger().info("Voice recognizer running in text_only mode (listening on /voice/command_text).")
            return

        cache_dir = Path(os.getenv('TB4_VOICE_CACHE', str(Path.home() / '.cache/turtlebot4_voice')))
        cache_dir.mkdir(parents=True, exist_ok=True)

        if self.backend_name == 'vosk':
            try:
                import vosk
                vosk.SetLogLevel(-1)

                model_dir = None
                if self.model_path and Path(self.model_path).is_dir():
                    model_dir = Path(self.model_path)
                else:
                    # Check default cache paths
                    candidates = [
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
                    self.get_logger().info(f"Loading Vosk model from: {model_dir}")
                    model = vosk.Model(str(model_dir))
                    self.vosk_recognizer = vosk.KaldiRecognizer(model, self.sample_rate)
                    self.get_logger().info("[Vosk] Model loaded successfully.")
                else:
                    self.get_logger().warn(
                        f"Vosk model not found at {cache_dir}. Run scripts/setup_environment.bash to download."
                        " Falling back to text_only bridge mode."
                    )
                    self.backend_name = 'text_only'
            except ImportError:
                self.get_logger().warn("Vosk library not installed. Falling back to text_only bridge mode.")
                self.backend_name = 'text_only'

        elif self.backend_name == 'whisper':
            try:
                import whisper
                self.get_logger().info("Loading Whisper 'tiny' model...")
                self.whisper_model = whisper.load_model('tiny')
                self.get_logger().info("[Whisper] Model loaded successfully.")
            except ImportError:
                self.get_logger().warn("Whisper library not installed. Falling back to text_only bridge mode.")
                self.backend_name = 'text_only'

    def _publish_status(self, state: str, message: str = ""):
        data = {
            "state": state,
            "backend": self.backend_name,
            "language": self.language,
            "message": message,
            "timestamp": time.time()
        }
        self.status_pub.publish(String(data=json.dumps(data)))

    def on_text_command(self, msg: String):
        """Direct text injection handler."""
        text = msg.data.strip()
        if text:
            self.get_logger().info(f"[Text Command Received]: '{text}'")
            self._publish_status("RECOGNIZED", text)
            self.text_pub.publish(String(data=text))

    def process_audio_file(self, wav_path: str) -> Optional[str]:
        """Transcribes a .wav file directly."""
        path = Path(wav_path).resolve()
        if not path.is_file():
            self.get_logger().error(f"Audio file not found: {path}")
            return None

        self.get_logger().info(f"Processing audio file: {path}")
        self._publish_status("PROCESSING_FILE", str(path))

        with wave.open(str(path), "rb") as wf:
            if wf.getnchannels() != 1 or wf.getsampwidth() != 2:
                self.get_logger().warn("Audio file must be mono 16-bit PCM WAV.")
            data = wf.readframes(wf.getnframes())

        transcribed = ""
        if self.vosk_recognizer:
            self.vosk_recognizer.Reset()
            self.vosk_recognizer.AcceptWaveform(data)
            res = json.loads(self.vosk_recognizer.FinalResult())
            transcribed = res.get('text', '')
        elif self.whisper_model:
            audio_arr = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
            result = self.whisper_model.transcribe(audio_arr, language=self.language)
            transcribed = result.get('text', '')

        if transcribed:
            self.get_logger().info(f"[File Transcribed]: '{transcribed}'")
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
                self.get_logger().warn(f"Audio stream status: {status}")
            if self.is_running:
                self.audio_queue.put(bytes(indata))

        dev_id = None if self.device_index < 0 else self.device_index

        try:
            with sd.RawInputStream(
                samplerate=self.sample_rate,
                blocksize=4000,
                device=dev_id,
                dtype='int16',
                channels=1,
                callback=audio_callback
            ):
                self.get_logger().info(f"Microphone stream active at {self.sample_rate} Hz.")
                while self.is_running and rclpy.ok():
                    try:
                        data = self.audio_queue.get(timeout=0.1)
                    except queue.Empty:
                        continue

                    if self.vosk_recognizer:
                        if self.vosk_recognizer.AcceptWaveform(data):
                            res = json.loads(self.vosk_recognizer.Result())
                            text = res.get('text', '').strip()
                            if text:
                                self.get_logger().info(f"[Vosk Recognized]: '{text}'")
                                self._publish_status("RECOGNIZED", text)
                                self.text_pub.publish(String(data=text))
                        else:
                            # Partial result can be checked if needed
                            pass
        except Exception as exc:
            self.get_logger().warn(f"Microphone stream error: {exc}. Text input remains available on {self.get_parameter('command_topic').value}")

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
