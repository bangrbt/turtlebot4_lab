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

# Initialize TrueType font for Vietnamese UTF-8 rendering on frames
_TTF_FONT_PATH = None
for _cand_font in [
    '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
    '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf',
    '/usr/share/fonts/truetype/freefont/FreeSansBold.ttf',
]:
    if os.path.isfile(_cand_font):
        _TTF_FONT_PATH = _cand_font
        break


def draw_text_utf8(
    image: np.ndarray,
    text: str,
    pos: Tuple[int, int],
    font_size: int = 16,
    color_bgr: Tuple[int, int, int] = (255, 255, 255),
    bg_color_bgr: Optional[Tuple[int, int, int]] = None,
    padding: int = 4
) -> np.ndarray:
    """Draw crisp Unicode / Vietnamese UTF-8 text onto a BGR numpy image using PIL."""
    if _TTF_FONT_PATH is None:
        # Fallback to OpenCV putText with ASCII translation
        import unicodedata
        ascii_text = ''.join(c for c in unicodedata.normalize('NFD', text) if unicodedata.category(c) != 'Mn')
        cv2.putText(image, ascii_text, pos, cv2.FONT_HERSHEY_SIMPLEX, font_size / 28.0, color_bgr, 1, cv2.LINE_AA)
        return image

    try:
        from PIL import Image, ImageDraw, ImageFont
        # Convert BGR to RGB PIL image
        pil_img = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil_img)
        font = ImageFont.truetype(_TTF_FONT_PATH, font_size)

        # Calculate bounding box of text
        x, y = pos
        bbox = draw.textbbox((x, y), text, font=font)
        if bg_color_bgr is not None:
            bg_rgb = (bg_color_bgr[2], bg_color_bgr[1], bg_color_bgr[0])
            draw.rectangle(
                [bbox[0] - padding, bbox[1] - padding, bbox[2] + padding, bbox[3] + padding],
                fill=bg_rgb
            )

        text_rgb = (color_bgr[2], color_bgr[1], color_bgr[0])
        draw.text((x, y), text, font=font, fill=text_rgb)

        # Convert back to BGR numpy array
        return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
    except Exception:
        import unicodedata
        ascii_text = ''.join(c for c in unicodedata.normalize('NFD', text) if unicodedata.category(c) != 'Mn')
        cv2.putText(image, ascii_text, pos, cv2.FONT_HERSHEY_SIMPLEX, font_size / 28.0, color_bgr, 1, cv2.LINE_AA)
        return image



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
    is_calibrated: bool = False
    calibrating: bool = False
    calibration_progress: int = 0

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
            'is_calibrated': self.is_calibrated,
            'calibrating': self.calibrating,
            'calibration_progress': self.calibration_progress,
        }


class EmotionDetector:
    """Robust Facial Emotion Detector with Multi-Modal CNN + Geometric Action Unit Fusion.
    Solves ambiguity between Fear vs Surprise, and Angry vs Resting Neutral.
    """

    def __init__(
        self,
        backend: str = 'auto',
        model_path: Optional[str] = None,
        min_face_size: int = 60,
        confidence_threshold: float = 0.35,
        smooth_window: int = 5,
        hysteresis_frames: int = 3,
    ):
        self.backend = backend
        self.min_face_size = min_face_size
        self.confidence_threshold = confidence_threshold
        self.smooth_window = max(1, smooth_window)
        self.hysteresis_frames = max(1, hysteresis_frames)

        # History for temporal smoothing
        self._history: deque = deque(maxlen=self.smooth_window)

        # Hysteresis state machine to prevent frame-to-frame jitter
        self._stable_emotion: str = 'neutral'
        self._candidate_emotion: str = 'neutral'
        self._candidate_count: int = 0

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

        # 3. High-precision MediaPipe FaceMesh for 3D Action Unit geometry
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

        # 4. Personal Face Baseline Calibrator (Quét hiệu chuẩn khuôn mặt cá nhân)
        self.calibrating: bool = False
        self.is_calibrated: bool = False
        self.calibration_target_frames: int = 40  # ~1.3 seconds at 30 FPS
        self.calibration_samples: List[Dict[str, float]] = []
        self.baseline: Dict[str, float] = {
            'brow_knit': 0.25,
            'brow_elevation': 0.060,
            'mouth_width_ratio': 0.32,
            'mar': 0.10,
            'smile_elevation': 0.0,
            'ear': 0.25,
        }

    def start_calibration(self, target_frames: int = 40) -> None:
        """Trigger personal face calibration: learn user's unique resting neutral geometry."""
        self.calibrating = True
        self.calibration_target_frames = max(5, target_frames)
        self.calibration_samples.clear()

    def reset_calibration(self) -> None:
        """Reset personal calibration back to population defaults."""
        self.calibrating = False
        self.is_calibrated = False
        self.calibration_samples.clear()
        self.baseline = {
            'brow_knit': 0.25,
            'brow_elevation': 0.060,
            'mouth_width_ratio': 0.32,
            'mar': 0.10,
            'smile_elevation': 0.0,
            'ear': 0.25,
        }

    def get_calibration_progress(self) -> float:
        """Return calibration progress from 0.0 to 1.0."""
        if not self.calibrating or self.calibration_target_frames <= 0:
            return 1.0 if self.is_calibrated else 0.0
        return min(1.0, len(self.calibration_samples) / float(self.calibration_target_frames))

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

    def _extract_geometric_features(self, landmarks, img_w: int, img_h: int) -> Dict[str, float]:
        """Extract scale-invariant Action Unit (AU) metrics from MediaPipe 468 landmarks."""
        lm = landmarks
        # Scale reference: Face Width (zygomatic arch) & Face Height
        fw = max(1e-3, abs(lm[454].x - lm[234].x))
        fh = max(1e-3, abs(lm[152].y - lm[10].y))

        # 1. Mouth Aspect Ratio (MAR: AU26/AU27 jaw drop) & Mouth Width Ratio (AU20 horizontal stretch)
        mw = max(1e-3, abs(lm[291].x - lm[61].x))
        mh = abs(lm[14].y - lm[13].y)
        mar = mh / mw
        mouth_width_ratio = mw / fw

        # 2. Inner Brow Knit (AU4 Corrugator supercilii - Glabella distance)
        # In anger, inner brows pull tightly together -> brow_knit decreases significantly (< 0.22)
        brow_knit = abs(lm[285].x - lm[55].x) / fw

        # 3. Eyebrow Elevation relative to eyes (AU1 + AU2 Frontalis)
        # In surprise, brows arch high (> 0.065); in anger, brows are lowered (< 0.048)
        left_brow_dist = lm[159].y - lm[70].y
        right_brow_dist = lm[386].y - lm[300].y
        brow_elevation = (left_brow_dist + right_brow_dist) / (2.0 * fh)

        # 4. Smile / Frown Elevation (AU12 Zygomaticus major vs AU15 Depressor anguli oris)
        # Measured relative to upper lip: In a smile, corners lift UP (corners_y < upper_lip_y -> positive elevation)
        upper_lip_y = (lm[0].y + lm[13].y) / 2.0
        corners_y = (lm[61].y + lm[291].y) / 2.0
        smile_elevation = (upper_lip_y - corners_y) / fh

        # 5. Eye Aspect Ratio (EAR: AU5 eye widener vs AU7 lid tightener)
        left_ear = abs(lm[145].y - lm[159].y) / max(1e-3, abs(lm[133].x - lm[33].x))
        right_ear = abs(lm[374].y - lm[386].y) / max(1e-3, abs(lm[263].x - lm[362].x))
        ear = (left_ear + right_ear) / 2.0

        return {
            'mar': mar,
            'mouth_width_ratio': mouth_width_ratio,
            'brow_knit': brow_knit,
            'brow_elevation': brow_elevation,
            'smile_elevation': smile_elevation,
            'ear': ear,
        }

    def _disambiguate_emotions(
        self,
        raw_probs: Dict[str, float],
        geom: Optional[Dict[str, float]]
    ) -> Dict[str, float]:
        """Smart Fusion: Respect decisive CNN predictions; use personal baseline deviations to resolve ambiguity."""
        probs = dict(raw_probs)
        if geom is None:
            return probs

        # Rank CNN predictions
        sorted_cnn = sorted(raw_probs.items(), key=lambda x: x[1], reverse=True)
        top_class, top_val = sorted_cnn[0]
        second_class, second_val = sorted_cnn[1]

        # 1. GOLDEN RULE: If CNN is decisively confident (> 60% and gap > 20%), trust CNN directly!
        if top_val >= 60.0 and (top_val - second_val) >= 20.0:
            return raw_probs

        # 2. Ambiguity resolution using deviations from user's personal baseline
        b = self.baseline
        d_mar = geom['mar'] - b.get('mar', 0.10)
        d_mw = geom['mouth_width_ratio'] - b.get('mouth_width_ratio', 0.32)
        d_knit = geom['brow_knit'] - b.get('brow_knit', 0.25)
        d_elev = geom['brow_elevation'] - b.get('brow_elevation', 0.060)
        d_smile = geom['smile_elevation'] - b.get('smile_elevation', 0.0)

        # Rule A: HAPPY (Corners lifted above personal baseline, mouth widened, not gaping)
        if d_smile > 0.007 and geom['mar'] < 0.25 and d_mw > 0.010:
            probs['happy'] += 40.0
            probs['sad'] *= 0.15
            probs['fear'] *= 0.15

        # Rule B: SAD (Corners droop below personal baseline)
        elif d_smile < -0.007 and geom['mar'] < 0.20:
            probs['sad'] += 40.0
            probs['happy'] *= 0.10

        # Rule C: SURPRISE vs FEAR
        # High MAR (gaping jaw) + wide popped eyes (high EAR) -> shock, surprise or horror/fear
        if d_mar > 0.20 and geom['ear'] > 0.30:
            if d_elev > 0.008 or raw_probs.get('surprise', 0.0) > raw_probs.get('fear', 0.0):
                probs['surprise'] += 45.0
                probs['fear'] *= 0.30
            else:
                probs['fear'] += 45.0
                probs['surprise'] *= 0.30
            probs['happy'] *= 0.05
            probs['sad'] *= 0.05
            probs['angry'] *= 0.10
            probs['neutral'] *= 0.05
        # Surprise: vertical jaw drop significantly higher than personal baseline
        elif d_mar > 0.18 and d_elev > -0.005:
            probs['surprise'] += 45.0
            probs['fear'] *= 0.20
            probs['happy'] *= 0.05
            probs['sad'] *= 0.05
        # Fear: mouth stretched horizontally wider than personal baseline without jaw drop
        elif d_mw > 0.04 and d_mar < 0.18 and geom['ear'] > 0.26:
            probs['fear'] += 40.0
            probs['surprise'] *= 0.25
            probs['happy'] *= 0.05
            probs['sad'] *= 0.05

        # Rule D: ANGRY vs NEUTRAL
        # Angry: brows knit significantly closer together than personal baseline (AU4 furrow)
        if d_knit < -0.025 and geom['mar'] < 0.22:
            probs['angry'] += 45.0
            probs['neutral'] *= 0.15
        elif d_knit >= -0.010 and geom['mar'] < 0.18:
            # Brows relaxed AND mouth closed -> resting state, suppress false resting anger
            probs['angry'] *= 0.20
            probs['neutral'] += 35.0

        # Rule E: NEUTRAL (facial features match personal resting baseline)
        if abs(d_smile) < 0.007 and geom['mar'] < 0.15 and abs(d_knit) < 0.020:
            probs['neutral'] += 35.0

        # Normalize to 100%
        total = sum(probs.values())
        if total > 0:
            return {k: round((v / total) * 100.0, 2) for k, v in probs.items()}
        return probs

    def analyze(self, bgr_image: np.ndarray) -> EmotionResult:
        """Analyze facial expression with bi-modal CNN + Landmark fusion and hysteresis."""
        if bgr_image is None or bgr_image.size == 0:
            return EmotionResult(emotion='neutral', face_detected=False)

        img_h, img_w = bgr_image.shape[:2]
        geom_features: Optional[Dict[str, float]] = None
        box: Optional[Tuple[int, int, int, int]] = None

        # Step 1: Run MediaPipe FaceMesh to extract 3D landmarks & precise bounding box
        if self.mp_face_mesh is not None:
            try:
                rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
                mesh_results = self.mp_face_mesh.process(rgb_image)
                if mesh_results.multi_face_landmarks:
                    lms = mesh_results.multi_face_landmarks[0].landmark
                    geom_features = self._extract_geometric_features(lms, img_w, img_h)

                    # Compute precise, jitter-free bounding box from landmarks
                    xs = [l.x for l in lms]
                    ys = [l.y for l in lms]
                    min_x, max_x = min(xs) * img_w, max(xs) * img_w
                    min_y, max_y = min(ys) * img_h, max(ys) * img_h
                    bw = max_x - min_x
                    bh = max_y - min_y

                    # 12% margin
                    pad_w, pad_h = 0.12 * bw, 0.12 * bh
                    bx = int(max(0, min_x - pad_w))
                    by = int(max(0, min_y - pad_h))
                    bw = int(min(img_w - bx, bw + 2 * pad_w))
                    bh = int(min(img_h - by, bh + 2 * pad_h))
                    box = (bx, by, bw, bh)
            except Exception:
                geom_features = None

        # Step 2: Fallback to OpenCV Haar Cascade if MediaPipe didn't produce a box
        if box is None:
            box = self.detect_face(bgr_image)

        if box is None:
            # No face detected
            self._history.clear()
            self._stable_emotion = 'neutral'
            self._candidate_emotion = 'neutral'
            self._candidate_count = 0
            if self.calibrating:
                prog = int((len(self.calibration_samples) / float(self.calibration_target_frames)) * 100)
                return EmotionResult(
                    emotion='neutral',
                    vietnamese=f'ĐANG QUÉT MẶT ({prog}%)',
                    confidence=prog / 100.0,
                    probabilities={k: 0.0 for k in TARGET_EMOTIONS},
                    action='CALIBRATING',
                    action_desc='Đang chờ khuôn mặt rõ ràng trong camera...',
                    face_detected=False,
                    face_box=None,
                    linear_x=0.0,
                    angular_z=0.0,
                    is_calibrated=self.is_calibrated,
                    calibrating=True,
                    calibration_progress=prog,
                )
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
                is_calibrated=self.is_calibrated,
                calibrating=False,
                calibration_progress=100 if self.is_calibrated else 0,
            )

        # Handle calibration mode (Quét khuôn mặt cá nhân)
        if self.calibrating:
            prog = int((len(self.calibration_samples) / float(self.calibration_target_frames)) * 100)
            if geom_features is not None:
                self.calibration_samples.append(geom_features)
                prog = min(100, int((len(self.calibration_samples) / float(self.calibration_target_frames)) * 100))
                if len(self.calibration_samples) >= self.calibration_target_frames:
                    keys = ['brow_knit', 'brow_elevation', 'mouth_width_ratio', 'mar', 'smile_elevation', 'ear']
                    self.baseline = {
                        k: float(np.mean([s[k] for s in self.calibration_samples if k in s]))
                        for k in keys
                    }
                    self.calibrating = False
                    self.is_calibrated = True

            self._history.clear()
            self._stable_emotion = 'neutral'
            self._candidate_emotion = 'neutral'
            self._candidate_count = 0

            return EmotionResult(
                emotion='neutral',
                vietnamese=f'ĐANG QUÉT MẶT ({prog}%)' if self.calibrating else 'ĐÃ QUÉT XONG',
                confidence=prog / 100.0,
                probabilities={'neutral': 100.0, 'happy': 0.0, 'sad': 0.0, 'angry': 0.0, 'surprise': 0.0, 'fear': 0.0},
                action='CALIBRATING',
                action_desc=f'Giữ khuôn mặt tự nhiên nhìn thẳng camera ({prog}%)' if self.calibrating else 'Đã ghi nhận cấu trúc mặt cá nhân!',
                face_detected=True,
                face_box=box,
                linear_x=0.0,
                angular_z=0.0,
                is_calibrated=self.is_calibrated,
                calibrating=self.calibrating,
                calibration_progress=prog,
            )

        x, y, w, h = box
        face_roi = bgr_image[y:y+h, x:x+w]
        if face_roi.size == 0:
            return EmotionResult(face_detected=True, face_box=box)

        # Step 3: Run DeepFace ONNX CNN with proper [0.0, 1.0] normalization
        raw_probs: Dict[str, float] = {}
        if self.onnx_session is not None:
            try:
                face_gray = cv2.cvtColor(face_roi, cv2.COLOR_BGR2GRAY)
                # Histogram equalization to enhance local facial contrast
                face_gray = cv2.equalizeHist(face_gray)
                face_48 = cv2.resize(face_gray, (48, 48), interpolation=cv2.INTER_AREA)

                # CRITICAL: Normalize to [0.0, 1.0] float32 as trained by DeepFace
                blob = (face_48.astype(np.float32) / 255.0)[np.newaxis, :, :, np.newaxis]

                raw_out = self.onnx_session.run(None, {'input': blob})[0][0]
                raw_dict = {label: float(val) for label, val in zip(ALL_MODEL_LABELS, raw_out)}

                # Map disgust into angry
                angry_val = raw_dict.get('angry', 0.0) + raw_dict.get('disgust', 0.0)

                extracted = {
                    'happy': raw_dict.get('happy', 0.0),
                    'sad': raw_dict.get('sad', 0.0),
                    'neutral': raw_dict.get('neutral', 0.0),
                    'angry': angry_val,
                    'surprise': raw_dict.get('surprise', 0.0),
                    'fear': raw_dict.get('fear', 0.0),
                }

                total = sum(extracted.values())
                if total > 0:
                    raw_probs = {k: (v / total) * 100.0 for k, v in extracted.items()}
            except Exception:
                raw_probs = {}

        # Default distribution if ONNX failed
        if not raw_probs:
            raw_probs = {'neutral': 100.0, 'happy': 0.0, 'sad': 0.0, 'angry': 0.0, 'surprise': 0.0, 'fear': 0.0}

        # Step 4: Multi-Modal Geometric Disambiguation
        refined_probs = self._disambiguate_emotions(raw_probs, geom_features)

        # Step 5: Temporal Sliding Window Smoothing
        self._history.append(refined_probs)
        smoothed_probs: Dict[str, float] = {}
        for emo in TARGET_EMOTIONS:
            smoothed_probs[emo] = float(np.mean([h[emo] for h in self._history]))

        # Step 6: Hysteresis State Machine (Jitter-free state switching)
        instant_dominant = max(smoothed_probs, key=smoothed_probs.get)
        instant_prob = smoothed_probs[instant_dominant] / 100.0
        current_prob = smoothed_probs.get(self._stable_emotion, 0.0) / 100.0

        if instant_dominant == self._stable_emotion:
            self._candidate_emotion = instant_dominant
            self._candidate_count = 0
        else:
            # If probability difference is very decisive (> 20% margin and > 60% confidence), switch faster
            if instant_prob >= 0.60 and (instant_prob - current_prob) >= 0.20:
                self._stable_emotion = instant_dominant
                self._candidate_emotion = instant_dominant
                self._candidate_count = 0
            elif instant_dominant == self._candidate_emotion:
                self._candidate_count += 1
                if self._candidate_count >= self.hysteresis_frames:
                    self._stable_emotion = instant_dominant
                    self._candidate_count = 0
            else:
                self._candidate_emotion = instant_dominant
                self._candidate_count = 1

        chosen_emotion = self._stable_emotion
        confidence = smoothed_probs[chosen_emotion] / 100.0

        if confidence < self.confidence_threshold:
            chosen_emotion = 'neutral'

        action_name, action_desc = ACTION_DESCRIPTIONS[chosen_emotion]
        vietnamese = EMOTION_VIETNAMESE[chosen_emotion]
        linear_x, angular_z = self._get_velocity_for_emotion(chosen_emotion)

        return EmotionResult(
            emotion=chosen_emotion,
            vietnamese=vietnamese,
            confidence=confidence,
            probabilities=smoothed_probs,
            action=action_name,
            action_desc=action_desc,
            face_detected=True,
            face_box=box,
            linear_x=linear_x,
            angular_z=angular_z,
            is_calibrated=self.is_calibrated,
            calibrating=False,
            calibration_progress=100 if self.is_calibrated else 0,
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

        # Top Overlay: Current Action and velocities
        overlay = vis.copy()
        cv2.rectangle(overlay, (0, 0), (w, 56), (20, 24, 32), -1)
        cv2.addWeighted(overlay, 0.75, vis, 0.25, 0, vis)

        if result.calibrating:
            action_str = f"ROBOT: DỪNG (ĐANG QUÉT MẶT CÁ NHÂN) | TIẾN ĐỘ: {result.calibration_progress}%"
            vis = draw_text_utf8(vis, action_str, (12, 6), font_size=15, color_bgr=(0, 220, 255))
        else:
            calib_tag = " [✓ Mặt cá nhân]" if result.is_calibrated else " [Chưa quét mặt]"
            action_str = f"ROBOT: {result.action} | {result.action_desc}{calib_tag}"
            vis = draw_text_utf8(vis, action_str, (12, 6), font_size=15, color_bgr=(0, 220, 255))

        vel_str = f"vx={result.linear_x:+.2f} m/s | wz={result.angular_z:+.2f} rad/s | FPS={fps:.1f}"
        cv2.putText(vis, vel_str, (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1, cv2.LINE_AA)

        # Draw Face Bounding Box
        if result.face_detected and result.face_box:
            bx, by, bw, bh = result.face_box

            if result.calibrating:
                # Calibration visual: vibrant gold/cyan bounding box with live progress bar
                calib_color = (255, 180, 0)
                cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), calib_color, 3)

                bar_y = by + bh + 6
                cv2.rectangle(vis, (bx, bar_y), (bx + bw, bar_y + 10), (50, 50, 50), -1)
                fill_w = int((result.calibration_progress / 100.0) * bw)
                if fill_w > 0:
                    cv2.rectangle(vis, (bx, bar_y), (bx + fill_w, bar_y + 10), (0, 220, 255), -1)

                label_text = f"🎯 ĐANG QUÉT MẶT: {result.calibration_progress}% (Giữ mặt thả lỏng)"
                label_y = max(60, by - 26)
                vis = draw_text_utf8(
                    vis,
                    label_text,
                    (bx + 4, label_y),
                    font_size=14,
                    color_bgr=(255, 255, 255),
                    bg_color_bgr=calib_color,
                    padding=3
                )
            else:
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

                # Top label banner with full Vietnamese UTF-8 support
                label_text = f"{result.emotion.upper()} ({int(result.confidence * 100)}%) — {result.vietnamese}"
                label_y = max(60, by - 26)
                vis = draw_text_utf8(
                    vis,
                    label_text,
                    (bx + 4, label_y),
                    font_size=14,
                    color_bgr=(255, 255, 255),
                    bg_color_bgr=border_color,
                    padding=3
                )

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
