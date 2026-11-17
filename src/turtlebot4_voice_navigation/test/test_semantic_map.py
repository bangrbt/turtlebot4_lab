"""Unit tests for semantic map loading, alias resolution, and point-in-polygon queries (5 zones)."""
from pathlib import Path
import pytest
from turtlebot4_voice_navigation.semantic_map import SemanticMap, point_in_polygon

@pytest.fixture
def semantic_map():
    pkg_dir = Path(__file__).resolve().parent.parent
    return SemanticMap(str(pkg_dir / 'config/zones.yaml'))

def test_zones_loaded(semantic_map):
    ids = semantic_map.get_zone_ids()
    assert ids == ['A', 'B', 'C', 'D', 'E']
    assert len(semantic_map.get_all_zones()) == 5

@pytest.mark.parametrize("query,expected_id", [
    ("A", "A"),
    ("khu vực a", "A"),
    ("phòng học", "A"),
    ("phong hoc", "A"),
    ("lớp học", "A"),
    ("classroom", "A"),
    ("B", "B"),
    ("khu vực b", "B"),
    ("phòng thí nghiệm", "B"),
    ("phong thi nghiem", "B"),
    ("lab", "B"),
    ("laboratory", "B"),
    ("C", "C"),
    ("khu vực c", "C"),
    ("văn phòng", "C"),
    ("van phong", "C"),
    ("phòng làm việc", "C"),
    ("office", "C"),
    ("D", "D"),
    ("khu vực d", "D"),
    ("thư viện", "D"),
    ("thu vien", "D"),
    ("phòng đọc", "D"),
    ("library", "D"),
    ("E", "E"),
    ("khu vực e", "E"),
    ("hành lang", "E"),
    ("hanh lang", "E"),
    ("sảnh", "E"),
    ("sảnh chờ", "E"),
    ("corridor", "E"),
    ("lobby", "E"),
])
def test_alias_lookup(semantic_map, query, expected_id):
    z = semantic_map.lookup_zone(query)
    assert z is not None, f"Failed to lookup '{query}'"
    assert z.id == expected_id

def test_invalid_zone_lookup(semantic_map):
    for bad in ["khu vực z", "nhà ăn", "căng tin", "toilet", "kitchen", "unknown", "phòng tổng thống"]:
        assert semantic_map.lookup_zone(bad) is None

@pytest.mark.parametrize("x,y,expected_zone", [
    (-3.2, 2.6, "A"),  # Inside Classroom
    (-4.0, 3.0, "A"),
    (3.2, 2.6, "B"),   # Inside Lab
    (4.0, 3.0, "B"),
    (3.2, -2.6, "C"),  # Inside Office
    (4.0, -3.0, "C"),
    (-3.2, -2.6, "D"), # Inside Library
    (-4.0, -3.0, "D"),
    (0.0, 0.0, "E"),   # Inside Lobby & Corridor
    (1.0, 0.0, "E"),
    (-2.0, 0.0, "E"),
])
def test_containment_and_queries(semantic_map, x, y, expected_zone):
    z = semantic_map.get_zone_at_point(x, y)
    assert z is not None, f"Point ({x}, {y}) should be in Zone {expected_zone}"
    assert z.id == expected_zone
