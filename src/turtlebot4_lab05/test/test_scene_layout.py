"""Scene geometry regression: centers alone must never approve a wall overlap."""
from copy import deepcopy
from pathlib import Path
import pytest
import yaml
from turtlebot4_lab05.scene_layout import validate_scene

PACKAGE = Path(__file__).resolve().parents[1]
try:
    from ament_index_python.packages import get_package_share_directory
    WORLD = Path(get_package_share_directory('turtlebot4_ignition_bringup'))/'worlds/maze.sdf'
    if not WORLD.is_file():
        raise FileNotFoundError
except Exception:
    WORLD = PACKAGE.parents[1]/'.cache_lab05/worlds/maze.sdf'


def check(data):
    return validate_scene(data, PACKAGE/'maps/map.yaml', WORLD)


def test_scattered_objects_clear_map_unknown_world_walls_and_each_other():
    result = check(yaml.safe_load((PACKAGE/'config/objects.yaml').read_text()))
    assert len(result['objects']) == 8
    assert all(min(o['map_body_gap_m'], o['world_wall_body_gap_m']) >= .35 for o in result['objects'])


def test_original_wall_touching_chair_is_rejected():
    data = yaml.safe_load((PACKAGE/'config/objects.yaml').read_text())
    chair = next(o for o in data['objects'] if o['name'] == 'lab05_chair_2')
    chair['pose'][:2] = [1.1, -3.8]
    with pytest.raises(ValueError, match='insufficient body-to-wall'):
        check(data)


def test_overlapping_objects_are_rejected():
    data = yaml.safe_load((PACKAGE/'config/objects.yaml').read_text())
    item = deepcopy(data['objects'][0])
    item['name'] = 'lab05_overlap'
    data['objects'].append(item)
    with pytest.raises(ValueError, match='too close'):
        check(data)
