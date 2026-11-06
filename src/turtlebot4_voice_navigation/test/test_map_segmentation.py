"""Unit tests for OpenCV map segmentation and coordinate transformations."""
from pathlib import Path
import pytest
from turtlebot4_voice_navigation.map_segmentation import (
    MapSegmenter, pixel_to_world, world_to_pixel
)

@pytest.fixture
def map_yaml():
    pkg_dir = Path(__file__).resolve().parent.parent
    return str(pkg_dir / 'maps/map.yaml')

def test_coordinate_round_trip():
    resolution = 0.05
    origin = [-7.0, -5.5, 0.0]
    height = 220

    test_points = [(0.0, 0.0), (-3.0, 2.0), (2.8, -2.0), (-5.5, 4.0)]
    for x, y in test_points:
        col, row = world_to_pixel(x, y, resolution, origin, height)
        rx, ry = pixel_to_world(col, row, resolution, origin, height)
        assert abs(rx - x) <= resolution
        assert abs(ry - y) <= resolution

def test_map_segmenter_load(map_yaml):
    seg = MapSegmenter(map_yaml)
    assert seg.width == 280
    assert seg.height == 220
    assert seg.resolution == 0.05
    assert seg.origin == [-7.0, -5.5, 0.0]

def test_free_space_extraction(map_yaml):
    seg = MapSegmenter(map_yaml)
    mask = seg.get_free_space_mask()
    assert mask.shape == (220, 280)
    free_pixels = int(mask.sum() / 255)
    assert free_pixels > 10000  # Substantial free navigable space

def test_watershed_segmentation(map_yaml):
    seg = MapSegmenter(map_yaml)
    markers, segments = seg.segment_watershed(min_room_area_m2=4.0)
    # Must segment at least 3 distinct rooms/zones as required by Lab 07
    assert len(segments) >= 3
    for s in segments:
        assert 'id' in s
        assert 'name' in s
        assert 'area_m2' in s and s['area_m2'] >= 4.0
        assert 'target_pose' in s
        assert 'clearance_m' in s and s['clearance_m'] > 0.4
        assert len(s['polygon']) >= 3
