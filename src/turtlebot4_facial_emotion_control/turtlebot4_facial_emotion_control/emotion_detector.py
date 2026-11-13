#!/usr/bin/env python3
"""Core Facial Emotion Detector for TurtleBot 4 Control.
Supports DeepFace ONNX weights, OpenCV Haar Cascade, and MediaPipe fallback.
"""
from collections import deque
from dataclasses import dataclass, field
import os
from pathlib import Path
import sys
from typing import Any, Dict, List, Optional, Tuple
import cv2
import numpy as np

# Inject workspace virtualenv site-packages if needed
for _cand in [
    Path('/home/anhbang/turtlebot4_lab_ws/.venv_gesture/lib/python3.10/site-packages'),
    Path.home() / 'turtlebot4_lab_ws/.venv_gesture/lib/python3.10/site-packages',
    Path(__file__).resolve().parents[4] / '.venv_gesture/lib/python3.10/site-packages',
    Path(__file__).resolve().parents[3] / '.venv_gesture/lib/python3.10/site-packages',
]:
    if _cand.is_dir() and str(_cand) not in sys.path:
        sys.path.insert(0, str(_cand))
        break

# Target 6 emotions specified in assignment
TARGET_EMOTIONS = ['happy', 'sad', 'neutral', 'angry', 'surprise', 'fear']

EMOTION_VIETNAMESE = {
    'happy': 'Vui vẻ',
    'sad': 'Buồn',
    'neutral': 'Bình thường',
    'angry': 'Tức giận',
    'surprise': 'Ngạc nhiên',
    'fear': 'Sợ hãi',
}

ACTION_DESCRIPTIONS = {
    'happy': ('HAPPY_CURVE_LEFT', 'Di chuyển theo quỹ đạo cong sang trái'),
    'sad': ('SAD_CURVE_RIGHT', 'Di chuyển theo quỹ đạo cong sang phải'),
    'neutral': ('NEUTRAL_STOP', 'Dừng tại chỗ'),
    'angry': ('ANGRY_REVERSE', 'Lùi nhẹ về phía sau'),
    'surprise': ('SURPRISE_SPIN', 'Xoay một vòng tại chỗ'),
    'fear': ('FEAR_FORWARD', 'Tiến nhẹ về phía trước'),
}

ALL_MODEL_LABELS = ['angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral']


@dataclass
class EmotionResult:
    """Structure holding the result of emotion detection."""
    emotion: str = 'neutral'
    vietnamese: str = 'Bình thường'
    confidence: float = 1.0
    probabilities: Dict[str, float] = field(default_factory=dict)
    action: str = 'NEUTRAL_STOP'
    action_desc: str = 'Dừng tại chỗ'
    face_detected: bool = False
    face_box: Optional[Tuple[int, int, int, int]] = None
    linear_x: float = 0.0
    angular_z: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            'emotion': self.emotion,
            'vietnamese': self.vietnamese,
            'confidence': round(self.confidence, 4),
            'probabilities': {k: round(v, 2) for k, v in self.probabilities.items()},
            'action': self.action,
            'action_desc': self.action_desc,
            'face_detected': self.face_detected,
            'face_box': list(self.face_box) if self.face_box else None,
            'linear_x': round(self.linear_x, 3),
            'angular_z': round(self.angular_z, 3),
        }


class EmotionDetector:
    """Robust Facial Emotion Detector with multi-tier engine support."""

    def __init__(
        self,
        backend: str = 'auto',
        model_path: Optional[str] = None,
        min_face_size: int = 60,
        confidence_threshold: float = 0.35,
        smooth_window: int = 5,
    ):
        self.backend = backend
        self.min_face_size = min_face_size
        self.confidence_threshold = confidence_threshold
        self.smooth_window = max(1, smooth_window)

        # History for temporal smoothing
        self._history: deque = deque(maxlen=self.smooth_window)

        # 1. Face Detector: OpenCV Haar Cascade
        haardir = getattr(getattr(cv2, 'data', None), 'haarcascades', '')
        cascade_candidates = [
            '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
            '/home/anhbang/turtlebot4_lab_ws/.venv_gesture/lib/python3.10/site-packages/cv2/data/haarcascade_frontalface_default.xml',
            str(Path.home() / 'turtlebot4_lab_ws/.venv_gesture/lib/python3.10/site-packages/cv2/data/haarcascade_frontalface_default.xml'),
            haardir + 'haarcascade_frontalface_default.xml' if haardir else '',
            '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml',
        ]
        self.face_cascade = None
        for p in cascade_candidates:
            if p and os.path.isfile(p):
                self.face_cascade = cv2.CascadeClassifier(p)
                break

        # 2. Emotion Model (ONNX weights from DeepFace)
        self.onnx_session = None
        self.model_loaded = False
        weights_candidates = [
            model_path,
            '/home/anhbang/turtlebot4_lab_ws/.cache_emotion/deepface/.deepface/weights/facial_expression_model_weights.onnx',
            str(Path.home() / 'turtlebot4_lab_ws/.cache_emotion/deepface/.deepface/weights/facial_expression_model_weights.onnx'),
            str(Path.home() / '.deepface/weights/facial_expression_model_weights.onnx'),
        ]

        # Load ONNX Runtime session if available
        try:
            import onnxruntime as ort
            sess_options = ort.SessionOptions()
            sess_options.log_severity_level = 3
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] if 'CUDAExecutionProvider' in ort.get_available_providers() else ['CPUExecutionProvider']

            for cand in weights_candidates:
                if cand and os.path.isfile(cand):
                    try:
                        self.onnx_session = ort.InferenceSession(cand, sess_options=sess_options, providers=providers)
                        self.model_loaded = True
                        break
                    except Exception:
                        pass
        except Exception:
            self.onnx_session = None

        # 3. MediaPipe fallback
        self.mp_face_mesh = None
        try:
            import mediapipe as mp
            self.mp_face_mesh = mp.solutions.face_mesh.FaceMesh(
                static_image_mode=False,
                max_num_faces=1,
                refine_landmarks=True,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
            )
        except Exception:
            self.mp_face_mesh = None

    def detect_face(self, bgr_image: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """Detect the largest frontal face bounding box (x, y, w, h)."""
        if bgr_image is None or bgr_image.size == 0 or self.face_cascade is None:
            return None

        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.15,
            minNeighbors=5,
            minSize=(self.min_face_size, self.min_face_size),
        )

        if len(faces) == 0:
            return None

        # Select largest face area
        faces = sorted(faces, key=lambda f: f[2] * f[3], reverse=True)
        x, y, w, h = faces[0]
        return int(x), int(y), int(w), int(h)

    def analyze(self, bgr_image: np.ndarray) -> EmotionResult:
        """Analyze facial expression in frame and return EmotionResult."""
        if bgr_image is None or bgr_image.size == 0:
            return EmotionResult(emotion='neutral', face_detected=False)

        box = self.detect_face(bgr_image)
        if box is None:
            # No face detected
            self._history.clear()
            action, desc = ACTION_DESCRIPTIONS['neutral']
            return EmotionResult(
                emotion='neutral',
                vietnamese='Không tìm thấy mặt',
                confidence=0.0,
                probabilities={k: 0.0 for k in TARGET_EMOTIONS},
                action='NEUTRAL_STOP',
                action_desc='Dừng tại chỗ (Mất dấu khuôn mặt)',
                face_detected=False,
                face_box=None,
                linear_x=0.0,
                angular_z=0.0,
            )

        x, y, w, h = box
        img_h, img_w = bgr_image.shape[:2]

        # Add 10% margin around face
        pad_x = int(0.10 * w)
        pad_y = int(0.10 * h)
        x1 = max(0, x - pad_x)
        y1 = max(0, y - pad_y)
        x2 = min(img_w, x + w + pad_x)
        y2 = min(img_h, y + h + pad_y)

        face_roi = bgr_image[y1:y2, x1:x2]
        if face_roi.size == 0:
            return EmotionResult(face_detected=True, face_box=box)

        # 1. Run ONNX Inference
        probs_dict: Dict[str, float] = {}
        if self.onnx_session is not None:
            try:
                face_gray = cv2.cvtColor(face_roi, cv2.COLOR_BGR2GRAY)
                face_48 = cv2.resize(face_gray, (48, 48), interpolation=cv2.INTER_AREA)
                blob = np.expand_dims(np.expand_dims(face_48, axis=0), axis=-1).astype(np.float32)

                raw_out = self.onnx_session.run(None, {'input': blob})[0][0]
                raw_dict = {label: float(val) for label, val in zip(ALL_MODEL_LABELS, raw_out)}

                # Map disgust into angry
                disgust_val = raw_dict.get('disgust', 0.0)
                angry_val = raw_dict.get('angry', 0.0) + disgust_val

                # Extract 6 target emotions
                extracted = {
                    'happy': raw_dict.get('happy', 0.0),
                    'sad': raw_dict.get('sad', 0.0),
                    'neutral': raw_dict.get('neutral', 0.0),
                    'angry': angry_val,
                    'surprise': raw_dict.get('surprise', 0.0),
                    'fear': raw_dict.get('fear', 0.0),
                }

                # Normalize to percentages summing to 100%
                total = sum(extracted.values())
                if total > 0:
                    probs_dict = {k: (v / total) * 100.0 for k, v in extracted.items()}
                else:
                    probs_dict = {k: (100.0 / 6.0) for k in TARGET_EMOTIONS}
            except Exception:
                probs_dict = {}

        # 2. MediaPipe Landmark Geometry Fallback (if ONNX didn't produce results)
        if not probs_dict and self.mp_face_mesh is not None:
            probs_dict = self._mediapipe_geometry_fallback(bgr_image)

        # 3. Default fallback if all fail
        if not probs_dict:
            probs_dict = {'neutral': 100.0, 'happy': 0.0, 'sad': 0.0, 'angry': 0.0, 'surprise': 0.0, 'fear': 0.0}

        # Apply Temporal Smoothing
        self._history.append(probs_dict)
        smoothed_probs: Dict[str, float] = {}
        for emo in TARGET_EMOTIONS:
            smoothed_probs[emo] = float(np.mean([h[emo] for h in self._history]))

        # Dominant emotion
        dominant_emotion = max(smoothed_probs, key=smoothed_probs.get)
        confidence = smoothed_probs[dominant_emotion] / 100.0

        # Confidence gate: If confidence is too low or neutral dominates
        if confidence < self.confidence_threshold:
            dominant_emotion = 'neutral'

        action_name, action_desc = ACTION_DESCRIPTIONS[dominant_emotion]
        vietnamese = EMOTION_VIETNAMESE[dominant_emotion]

        # Velocity mapping
        linear_x, angular_z = self._get_velocity_for_emotion(dominant_emotion)

        return EmotionResult(
            emotion=dominant_emotion,
            vietnamese=vietnamese,
            confidence=confidence,
            probabilities=smoothed_probs,
            action=action_name,
            action_desc=action_desc,
            face_detected=True,
            face_box=box,
            linear_x=linear_x,
            angular_z=angular_z,
        )

    def _get_velocity_for_emotion(self, emotion: str) -> Tuple[float, float]:
        """Return (linear_x, angular_z) based on the assignment rules."""
        if emotion == 'happy':
            # Happy: Quỹ đạo cong sang trái
            return 0.20, +0.60
        elif emotion == 'sad':
            # Sad: Quỹ đạo cong sang phải
            return 0.20, -0.60
        elif emotion == 'neutral':
            # Neutral: Dừng tại chỗ
            return 0.00, 0.00
        elif emotion == 'angry':
            # Angry: Lùi nhẹ về phía sau
            return -0.15, 0.00
        elif emotion == 'surprise':
            # Surprise: Xoay một vòng tại chỗ
            return 0.00, +1.20
        elif emotion == 'fear':
            # Fear: Tiến nhẹ về phía trước
            return +0.18, 0.00
        return 0.00, 0.00

    def _mediapipe_geometry_fallback(self, image_bgr: np.ndarray) -> Dict[str, float]:
        """Calculate geometric emotion heuristic from facial landmarks."""
        try:
            image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            results = self.mp_face_mesh.process(image_rgb)
            if not results.multi_face_landmarks:
                return {'neutral': 100.0, 'happy': 0.0, 'sad': 0.0, 'angry': 0.0, 'surprise': 0.0, 'fear': 0.0}

            lm = results.multi_face_landmarks[0].landmark
            # Landmark indices:
            # Mouth corners: 61 (left), 291 (right)
            # Upper/lower lips: 0, 17
            # Eyebrows: 70 (left inner), 300 (right inner)
            # Eyes: 159 (left top), 386 (right top)
            mouth_width = abs(lm[291].x - lm[61].x)
            mouth_height = abs(lm[17].y - lm[0].y)
            mouth_open_ratio = mouth_height / (mouth_width + 1e-6)

            # Smile: mouth corners elevation relative to upper lip
            corner_y = (lm[61].y + lm[291].y) / 2.0
            smile_score = lm[0].y - corner_y

            # Eyebrow raise
            brow_y = (lm[70].y + lm[300].y) / 2.0
            eye_y = (lm[159].y + lm[386].y) / 2.0
            brow_dist = eye_y - brow_y

            probs = {k: 5.0 for k in TARGET_EMOTIONS}
            if smile_score > 0.025:
                probs['happy'] = 80.0
            elif mouth_open_ratio > 0.40 and brow_dist > 0.06:
                probs['surprise'] = 80.0
            elif brow_dist < 0.035:
                probs['angry'] = 75.0
            elif smile_score < -0.015:
                probs['sad'] = 75.0
            elif mouth_open_ratio > 0.25 and brow_dist < 0.045:
                probs['fear'] = 70.0
            else:
                probs['neutral'] = 85.0

            total = sum(probs.values())
            return {k: (v / total) * 100.0 for k, v in probs.items()}
        except Exception:
            return {'neutral': 100.0, 'happy': 0.0, 'sad': 0.0, 'angry': 0.0, 'surprise': 0.0, 'fear': 0.0}

    def annotate_frame(
        self,
        frame: np.ndarray,
        result: EmotionResult,
        fps: float = 0.0,
    ) -> np.ndarray:
        """Draw bounding box, emotion tags, bars, and velocity arrows on frame."""
        vis = frame.copy()
        h, w = vis.shape[:2]

        # Draw Face Bounding Box
        if result.face_detected and result.face_box:
            bx, by, bw, bh = result.face_box
            # Choose color based on emotion
            color_map = {
                'happy': (46, 204, 113),      # Emerald Green
                'sad': (231, 76, 60),         # Alizarin Red/Blue
                'neutral': (149, 165, 166),   # Gray
                'angry': (0, 0, 220),         # Vivid Crimson
                'surprise': (243, 156, 18),   # Orange
                'fear': (155, 89, 182),       # Amethyst Purple
            }
            border_color = color_map.get(result.emotion, (0, 255, 0))

            # Bounding box
            cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), border_color, 2)

            # Top label banner
            label_text = f"{result.emotion.upper()} ({int(result.confidence * 100)}%) - {result.vietnamese}"
            (tw, th), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
            cv2.rectangle(vis, (bx, by - th - 12), (bx + tw + 16, by), border_color, -1)
            cv2.putText(vis, label_text, (bx + 8, by - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)

        # Top Overlay: Current Action and velocities
        overlay = vis.copy()
        cv2.rectangle(overlay, (0, 0), (w, 52), (20, 24, 32), -1)
        cv2.addWeighted(overlay, 0.75, vis, 0.25, 0, vis)

        action_str = f"ROBOT: {result.action} | {result.action_desc}"
        cv2.putText(vis, action_str, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 220, 0), 2, cv2.LINE_AA)

        vel_str = f"vx={result.linear_x:+.2f} m/s | wz={result.angular_z:+.2f} rad/s | FPS={fps:.1f}"
        cv2.putText(vis, vel_str, (12, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1, cv2.LINE_AA)

        # Left Overlay: Emotion Probabilities HUD
        hud_y = 70
        for emo in TARGET_EMOTIONS:
            pct = result.probabilities.get(emo, 0.0)
            is_dominant = (emo == result.emotion and result.face_detected)

            # Background bar
            cv2.rectangle(vis, (12, hud_y), (150, hud_y + 14), (40, 44, 52), -1)
            # Fill bar
            bar_w = int((pct / 100.0) * 138)
            fill_color = (0, 220, 0) if is_dominant else (100, 120, 140)
            if bar_w > 0:
                cv2.rectangle(vis, (12, hud_y), (12 + bar_w, hud_y + 14), fill_color, -1)

            text_color = (255, 255, 255) if is_dominant else (180, 180, 180)
            tag = f"{emo[:3].upper()}: {pct:4.1f}%"
            cv2.putText(vis, tag, (155, hud_y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.40, text_color, 1, cv2.LINE_AA)
            hud_y += 20

        return vis
