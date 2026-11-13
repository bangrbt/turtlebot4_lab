#!/usr/bin/env python3
"""Unit tests for EmotionDetector logic and motion mappings."""
import numpy as np
import pytest
from turtlebot4_facial_emotion_control.emotion_detector import (
    EmotionDetector,
    TARGET_EMOTIONS,
    ACTION_DESCRIPTIONS,
    EMOTION_VIETNAMESE,
)


def test_target_emotions_coverage():
    """Verify all 6 required emotions are mapped."""
    required = {'happy', 'sad', 'neutral', 'angry', 'surprise', 'fear'}
    assert set(TARGET_EMOTIONS) == required
    assert len(ACTION_DESCRIPTIONS) == 6
    assert len(EMOTION_VIETNAMESE) == 6


def test_velocity_mapping_rules():
    """Verify velocity matches the assignment requirements exactly."""
    detector = EmotionDetector(backend='auto')

    # 1. Happy: Quỹ đạo cong sang trái (linear > 0, angular > 0)
    vx, wz = detector._get_velocity_for_emotion('happy')
    assert vx > 0.0, "Happy must move forward"
    assert wz > 0.0, "Happy must turn left"

    # 2. Sad: Quỹ đạo cong sang phải (linear > 0, angular < 0)
    vx, wz = detector._get_velocity_for_emotion('sad')
    assert vx > 0.0, "Sad must move forward"
    assert wz < 0.0, "Sad must turn right"

    # 3. Neutral: Dừng tại chỗ (linear == 0, angular == 0)
    vx, wz = detector._get_velocity_for_emotion('neutral')
    assert vx == 0.0, "Neutral must stop linear motion"
    assert wz == 0.0, "Neutral must stop angular motion"

    # 4. Angry: Lùi nhẹ về phía sau (linear < 0, angular == 0)
    vx, wz = detector._get_velocity_for_emotion('angry')
    assert vx < 0.0, "Angry must reverse gently"
    assert wz == 0.0, "Angry must reverse straight"

    # 5. Surprise: Xoay một vòng tại chỗ (linear == 0, angular != 0)
    vx, wz = detector._get_velocity_for_emotion('surprise')
    assert vx == 0.0, "Surprise must not translate"
    assert abs(wz) > 0.5, "Surprise must spin in place"

    # 6. Fear: Tiến nhẹ về phía trước (linear > 0, angular == 0)
    vx, wz = detector._get_velocity_for_emotion('fear')
    assert vx > 0.0, "Fear must advance forward gently"
    assert wz == 0.0, "Fear must advance straight"


def test_detector_on_blank_image():
    """Blank image should yield no face detected and neutral stop."""
    detector = EmotionDetector(backend='auto')
    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    res = detector.analyze(blank)

    assert not res.face_detected
    assert res.emotion == 'neutral'
    assert res.linear_x == 0.0
    assert res.angular_z == 0.0
    assert res.action == 'NEUTRAL_STOP'


def test_detector_smoothing_and_normalization():
    """Verify probabilities sum to 100% and smoothing history functions."""
    detector = EmotionDetector(backend='auto', smooth_window=3)
    dummy_face = np.full((120, 120, 3), 180, dtype=np.uint8)

    # Analyze multiple times to test smoothing buffer
    for _ in range(5):
        res = detector.analyze(dummy_face)
        assert isinstance(res.probabilities, dict)
        if res.face_detected:
            prob_sum = sum(res.probabilities.values())
            assert abs(prob_sum - 100.0) < 1.0
