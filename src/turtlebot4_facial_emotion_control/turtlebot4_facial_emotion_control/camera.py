#!/usr/bin/env python3
"""Webcam stream manager and Mock Face Generator."""
import math
import os
import time
from typing import Optional, Tuple
import cv2
import numpy as np


class CameraManager:
    """Manages OpenCV VideoCapture with auto-fallback to synthetic face generator."""

    def __init__(
        self,
        camera_index: int = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        mirror: bool = True,
        use_mock_if_failed: bool = True,
    ):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.mirror = mirror
        self.use_mock = False
        self.cap: Optional[cv2.VideoCapture] = None

        # Check /dev/video*
        dev_path = f"/dev/video{camera_index}"
        if not os.path.exists(dev_path) and use_mock_if_failed:
            self.use_mock = True
        else:
            self._init_camera()

        if (self.cap is None or not self.cap.isOpened()) and use_mock_if_failed:
            self.use_mock = True

        # Mock animation state
        self._mock_start_time = time.monotonic()
        self.mock_forced_emotion: Optional[str] = None

    def _init_camera(self) -> None:
        try:
            self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(self.camera_index)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        except Exception:
            self.cap = None

    def read_frame(self) -> Tuple[bool, np.ndarray]:
        """Read a single frame. Returns (success, BGR image)."""
        if not self.use_mock and self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None and frame.size > 0:
                if self.mirror:
                    frame = cv2.flip(frame, 1)
                return True, frame

        # Use mock synthetic face generator
        return True, self._generate_mock_frame()

    def set_mock_emotion(self, emotion: Optional[str]) -> None:
        """Force the mock generator to render a specific emotion."""
        self.mock_forced_emotion = emotion

    def _generate_mock_frame(self) -> np.ndarray:
        """Draw an expressive synthetic face suitable for OpenCV Haar & emotion model."""
        img = np.full((self.height, self.width, 3), (240, 245, 250), dtype=np.uint8)
        cx, cy = self.width // 2, self.height // 2

        # Cycle through emotions every 4 seconds if not forced
        t = time.monotonic() - self._mock_start_time
        emotions_cycle = ['neutral', 'happy', 'sad', 'angry', 'surprise', 'fear']
        cycle_idx = int(t / 4.0) % len(emotions_cycle)
        current_emo = self.mock_forced_emotion or emotions_cycle[cycle_idx]

        # Draw Head / Face oval
        cv2.ellipse(img, (cx, cy), (120, 160), 0, 0, 360, (210, 225, 245), -1)
        cv2.ellipse(img, (cx, cy), (120, 160), 0, 0, 360, (140, 160, 190), 3)

        # Eyes positions
        eye_y = cy - 35
        left_eye_x = cx - 45
        right_eye_x = cx + 45

        # Eyebrows & Eyes according to emotion
        if current_emo == 'happy':
            # Eyes crinkled / smiling arcs
            cv2.ellipse(img, (left_eye_x, eye_y), (14, 10), 0, 180, 360, (50, 50, 70), 3)
            cv2.ellipse(img, (right_eye_x, eye_y), (14, 10), 0, 180, 360, (50, 50, 70), 3)
            # Upcurved eyebrows
            cv2.line(img, (left_eye_x - 20, eye_y - 20), (left_eye_x + 20, eye_y - 28), (40, 40, 60), 4)
            cv2.line(img, (right_eye_x - 20, eye_y - 28), (right_eye_x + 20, eye_y - 20), (40, 40, 60), 4)
            # Big smile
            cv2.ellipse(img, (cx, cy + 50), (45, 30), 0, 0, 180, (20, 20, 160), -1)
            cv2.ellipse(img, (cx, cy + 50), (45, 30), 0, 0, 180, (40, 40, 60), 3)
        elif current_emo == 'sad':
            # Normal eyes
            cv2.circle(img, (left_eye_x, eye_y), 10, (50, 50, 70), -1)
            cv2.circle(img, (right_eye_x, eye_y), 10, (50, 50, 70), -1)
            # Inner eyebrows raised (sad brow)
            cv2.line(img, (left_eye_x - 20, eye_y - 20), (left_eye_x + 20, eye_y - 30), (40, 40, 60), 4)
            cv2.line(img, (right_eye_x - 20, eye_y - 30), (right_eye_x + 20, eye_y - 20), (40, 40, 60), 4)
            # Downward mouth
            cv2.ellipse(img, (cx, cy + 70), (40, 25), 0, 180, 360, (40, 40, 60), 4)
        elif current_emo == 'angry':
            # Narrow eyes
            cv2.ellipse(img, (left_eye_x, eye_y), (12, 6), 0, 0, 360, (50, 50, 70), -1)
            cv2.ellipse(img, (right_eye_x, eye_y), (12, 6), 0, 0, 360, (50, 50, 70), -1)
            # Sharp V-shape eyebrows
            cv2.line(img, (left_eye_x - 20, eye_y - 30), (left_eye_x + 20, eye_y - 18), (20, 20, 140), 5)
            cv2.line(img, (right_eye_x - 20, eye_y - 18), (right_eye_x + 20, eye_y - 30), (20, 20, 140), 5)
            # Tense flat or slight downward mouth
            cv2.line(img, (cx - 35, cy + 55), (cx + 35, cy + 55), (40, 40, 60), 4)
        elif current_emo == 'surprise':
            # Wide open round eyes
            cv2.circle(img, (left_eye_x, eye_y), 16, (50, 50, 70), 3)
            cv2.circle(img, (left_eye_x, eye_y), 6, (50, 50, 70), -1)
            cv2.circle(img, (right_eye_x, eye_y), 16, (50, 50, 70), 3)
            cv2.circle(img, (right_eye_x, eye_y), 6, (50, 50, 70), -1)
            # High arched eyebrows
            cv2.ellipse(img, (left_eye_x, eye_y - 25), (22, 10), 0, 180, 360, (40, 40, 60), 4)
            cv2.ellipse(img, (right_eye_x, eye_y - 25), (22, 10), 0, 180, 360, (40, 40, 60), 4)
            # Open round mouth (O shape)
            cv2.ellipse(img, (cx, cy + 60), (20, 35), 0, 0, 360, (20, 20, 160), -1)
            cv2.ellipse(img, (cx, cy + 60), (20, 35), 0, 0, 360, (40, 40, 60), 3)
        elif current_emo == 'fear':
            # Wide eyes looking slightly askance
            cv2.circle(img, (left_eye_x, eye_y), 14, (50, 50, 70), 3)
            cv2.circle(img, (left_eye_x - 2, eye_y), 5, (50, 50, 70), -1)
            cv2.circle(img, (right_eye_x, eye_y), 14, (50, 50, 70), 3)
            cv2.circle(img, (right_eye_x - 2, eye_y), 5, (50, 50, 70), -1)
            # Raised flat eyebrows
            cv2.line(img, (left_eye_x - 20, eye_y - 28), (left_eye_x + 20, eye_y - 28), (40, 40, 60), 4)
            cv2.line(img, (right_eye_x - 20, eye_y - 28), (right_eye_x + 20, eye_y - 28), (40, 40, 60), 4)
            # Open trembling mouth
            cv2.ellipse(img, (cx, cy + 55), (30, 15), 0, 0, 360, (40, 40, 60), 3)
        else:
            # Neutral: calm eyes, straight brows, gentle relaxed mouth
            cv2.circle(img, (left_eye_x, eye_y), 10, (50, 50, 70), -1)
            cv2.circle(img, (right_eye_x, eye_y), 10, (50, 50, 70), -1)
            cv2.line(img, (left_eye_x - 20, eye_y - 22), (left_eye_x + 20, eye_y - 22), (40, 40, 60), 3)
            cv2.line(img, (right_eye_x - 20, eye_y - 22), (right_eye_x + 20, eye_y - 22), (40, 40, 60), 3)
            cv2.line(img, (cx - 25, cy + 50), (cx + 25, cy + 50), (40, 40, 60), 3)

        # Subtle noise / shading so face detector sees realistic contrast
        noise = np.random.normal(0, 3, img.shape).astype(np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # Watermark
        tag = f"[MOCK CAMERA] Mode: {current_emo.upper()} (Tự động mô phỏng khi không có webcam)"
        cv2.putText(img, tag, (20, self.height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (120, 140, 160), 1, cv2.LINE_AA)

        return img

    def release(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None
