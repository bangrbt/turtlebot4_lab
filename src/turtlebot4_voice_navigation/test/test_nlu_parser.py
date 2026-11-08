"""Unit tests for Natural Language Understanding (NLU) & Intent Parser (5 zones)."""
from pathlib import Path
import pytest
from turtlebot4_voice_navigation.semantic_map import SemanticMap
from turtlebot4_voice_navigation.nlu_parser import CommandParser, IntentType

@pytest.fixture
def parser():
    pkg_dir = Path(__file__).resolve().parent.parent
    sm = SemanticMap(str(pkg_dir / 'config/zones.yaml'))
    return CommandParser(sm)

@pytest.mark.parametrize("sentence,expected_target_id", [
    ("Đi đến khu vực A.", "A"),
    ("đi tới phòng học", "A"),
    ("di chuyển đến lớp học", "A"),
    ("hãy đến phòng a", "A"),
    ("Đi đến phòng thí nghiệm.", "B"),
    ("tới khu vực b", "B"),
    ("chạy tới phòng lab", "B"),
    ("navigate to laboratory", "B"),
    ("Đi đến văn phòng.", "C"),
    ("tới phòng c", "C"),
    ("go to office", "C"),
    ("Đi đến thư viện.", "D"),
    ("di chuyển tới phòng đọc", "D"),
    ("go to library", "D"),
    ("Đi đến hành lang.", "E"),
    ("di chuyển tới sảnh", "E"),
    ("go to corridor", "E"),
])
def test_direct_navigation(parser, sentence, expected_target_id):
    res = parser.parse(sentence)
    assert res.is_valid, res.error_message
    assert res.intent_type == IntentType.NAVIGATE_TO
    assert res.target_zone is not None
    assert res.target_zone.id == expected_target_id

@pytest.mark.parametrize("sentence,expected_orig_id,expected_dest_id", [
    ("Di chuyển từ khu vực A đến khu vực B.", "A", "B"),
    ("Đi từ phòng học đến văn phòng.", "A", "C"),
    ("Đi từ phòng thí nghiệm tới thư viện.", "B", "D"),
    ("navigate from office to laboratory", "C", "B"),
    ("từ sảnh sang phòng học", "E", "A"),
])
def test_from_to_navigation(parser, sentence, expected_orig_id, expected_dest_id):
    res = parser.parse(sentence)
    assert res.is_valid, res.error_message
    assert res.intent_type == IntentType.NAVIGATE_FROM_TO
    assert res.origin_zone.id == expected_orig_id
    assert res.target_zone.id == expected_dest_id
    assert len(res.waypoint_zones) == 2

def test_multi_hop_navigation(parser):
    sentence = "Đi từ phòng A qua hành lang E rồi đến phòng B"
    res = parser.parse(sentence)
    assert res.is_valid, res.error_message
    assert res.intent_type == IntentType.NAVIGATE_WAYPOINTS
    assert [z.id for z in res.waypoint_zones] == ["A", "E", "B"]

@pytest.mark.parametrize("sentence", [
    "Dừng lại",
    "dừng",
    "hủy lệnh",
    "hủy bỏ",
    "stop",
    "cancel",
    "halt",
])
def test_stop_commands(parser, sentence):
    res = parser.parse(sentence)
    assert res.is_valid
    assert res.intent_type == IntentType.STOP

@pytest.mark.parametrize("sentence", [
    "Robot đang ở đâu?",
    "ở đâu",
    "vị trí hiện tại",
    "where are you",
    "current location",
])
def test_location_query(parser, sentence):
    res = parser.parse(sentence)
    assert res.is_valid
    assert res.intent_type == IntentType.QUERY_LOCATION

@pytest.mark.parametrize("sentence,bad_part", [
    ("Đi đến khu vực Z", "khu vực z"),
    ("Đi đến nhà ăn", "nhà ăn"),
    ("Di chuyển tới căng tin", "căng tin"),
    ("Đi từ phòng A đến phòng Z", "phòng z"),
    ("go to cafeteria", "cafeteria"),
    ("Đi đến phòng tổng thống", "phòng tổng thống"),
])
def test_invalid_zones_rejected(parser, sentence, bad_part):
    res = parser.parse(sentence)
    assert not res.is_valid
    assert res.intent_type == IntentType.INVALID_ZONE
    assert len(res.error_message) > 0
    assert "không tồn tại" in res.error_message.lower() or "not exist" in res.error_message.lower()
