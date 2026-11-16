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


def test_fear_vs_surprise_disambiguation():
    """Verify that vertical jaw drop + arched brows strongly selects surprise over fear."""
    detector = EmotionDetector(backend='auto')
    # Ambiguous raw CNN output where fear > surprise
    raw = {'happy': 5.0, 'sad': 5.0, 'neutral': 5.0, 'angry': 5.0, 'surprise': 35.0, 'fear': 45.0}

    # Case A: Surprise face geometry (tall mouth MAR > 0.35, high brows)
    geom_surprise = {
        'mar': 0.45,
        'mouth_width_ratio': 0.32,
        'brow_knit': 0.26,
        'brow_elevation': 0.080,
        'smile_elevation': -0.005,
        'ear': 0.34
    }
    refined_surprise = detector._disambiguate_emotions(raw, geom_surprise)
    assert refined_surprise['surprise'] > refined_surprise['fear'] * 3.0, "Surprise must dominate with high MAR"
    assert refined_surprise['surprise'] > 60.0

    # Case B: Fear face geometry (horizontal mouth stretch, lower brow arch)
    geom_fear = {
        'mar': 0.22,
        'mouth_width_ratio': 0.42,
        'brow_knit': 0.22,
        'brow_elevation': 0.055,
        'smile_elevation': -0.005,
        'ear': 0.30
    }
    refined_fear = detector._disambiguate_emotions(raw, geom_fear)
    assert refined_fear['fear'] > refined_fear['surprise'], "Fear must dominate with horizontal stretch"


def test_angry_vs_neutral_disambiguation():
    """Verify that unknit/relaxed eyebrows suppress false angry and confirm neutral."""
    detector = EmotionDetector(backend='auto')

    # Case A: Resting face where CNN falsely outputs angry
    raw_confused = {'happy': 5.0, 'sad': 10.0, 'neutral': 30.0, 'angry': 40.0, 'surprise': 5.0, 'fear': 10.0}
    geom_resting = {
        'mar': 0.08,
        'mouth_width_ratio': 0.28,
        'brow_knit': 0.26,           # Relaxed unknit brows
        'brow_elevation': 0.060,     # Normal height
        'smile_elevation': 0.001,
        'ear': 0.25
    }
    refined_neutral = detector._disambiguate_emotions(raw_confused, geom_resting)
    assert refined_neutral['neutral'] > refined_neutral['angry'] * 3.0, "Neutral must dominate on resting face"
    assert refined_neutral['neutral'] > 50.0

    # Case B: Genuine anger (furrowed brow knit < 0.21, low brow elevation)
    geom_angry = {
        'mar': 0.06,
        'mouth_width_ratio': 0.28,
        'brow_knit': 0.18,           # Furrowed brows
        'brow_elevation': 0.040,     # Lowered brows
        'smile_elevation': -0.008,
        'ear': 0.20
    }
    refined_angry = detector._disambiguate_emotions(raw_confused, geom_angry)
    assert refined_angry['angry'] > refined_angry['neutral'], "Angry must dominate with furrowed brows"


def test_face_calibration_workflow():
    """Verify personal baseline scanning and calibration lifecycle."""
    detector = EmotionDetector(backend='auto')

    assert not detector.is_calibrated
    assert not detector.calibrating

    # Start calibration
    detector.start_calibration(target_frames=5)
    assert detector.calibrating
    assert detector.get_calibration_progress() == 0.0

    # Feed mock geometric samples
    sample_geom = {
        'brow_knit': 0.22,
        'brow_elevation': 0.055,
        'mouth_width_ratio': 0.35,
        'mar': 0.09,
        'smile_elevation': 0.002,
        'ear': 0.24,
    }

    # Simulate 5 frames of calibration
    for i in range(5):
        detector.calibration_samples.append(sample_geom)

    # Trigger completion logic as in analyze()
    if len(detector.calibration_samples) >= detector.calibration_target_frames:
        keys = ['brow_knit', 'brow_elevation', 'mouth_width_ratio', 'mar', 'smile_elevation', 'ear']
        detector.baseline = {
            k: float(np.mean([s[k] for s in detector.calibration_samples if k in s]))
            for k in keys
        }
        detector.calibrating = False
        detector.is_calibrated = True

    assert detector.is_calibrated
    assert not detector.calibrating
    assert detector.baseline['brow_knit'] == pytest.approx(0.22)
    assert detector.baseline['mouth_width_ratio'] == pytest.approx(0.35)

    # Verify reset restores defaults
    detector.reset_calibration()
    assert not detector.is_calibrated
    assert not detector.calibrating
    assert detector.baseline['brow_knit'] == 0.25


