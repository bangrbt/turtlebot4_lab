"""Synthetic box unit tests only; no camera, model inference, or robot demo."""
import pytest
from turtlebot4_lab05.detection_filter import DetectionConfirmation


def box(label=0, x=0, confidence=.9):
    return dict(class_id=label, confidence=confidence, xyxy=[x,0,x+100,100])


def test_requires_adjacent_consistent_predictions_and_preserves_current_indices():
    f = DetectionConfirmation()
    assert f.update([box()], 1) == []
    assert f.update([box(x=5)], 1.1) == [0]
    assert f.update([box(label=1,x=5)], 1.2) == []
    assert f.update([box(label=1,x=8)], 1.3) == [0]


def test_missing_detection_and_spatial_jump_break_confirmation():
    f = DetectionConfirmation()
    f.update([box()], 1)
    assert f.update([box(x=300)], 1.1) == []
    assert f.update([], 1.2) == []
    assert f.update([box(x=300)], 1.3) == []


def test_duplicate_stamp_cannot_confirm_a_frozen_image():
    f = DetectionConfirmation()
    f.update([box()], 1)
    assert f.update([box()], 1) == []


def test_wall_timeout_resets_even_when_simulation_clock_barely_moves():
    f = DetectionConfirmation(timeout=1.5)
    f.update([box()], 1, wall_time=10)
    assert f.update([box()], 1.01, wall_time=13) == []


def test_one_previous_box_cannot_confirm_two_new_objects():
    f = DetectionConfirmation()
    f.update([box()], 1)
    assert f.update([box(confidence=.95),box(x=10,confidence=.8)], 1.1) == [0]


@pytest.mark.parametrize('values', [dict(frames=0),dict(iou=0),dict(timeout=-1)])
def test_invalid_configuration_is_rejected(values):
    with pytest.raises(ValueError):
        DetectionConfirmation(**values)
