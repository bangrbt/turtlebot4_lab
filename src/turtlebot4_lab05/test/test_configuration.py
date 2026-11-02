from pathlib import Path
import math
import yaml
import pytest
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from turtlebot4_lab05.configuration import load_waypoints, point_is_free, quaternion_from_yaw

SHARE = Path(__file__).resolve().parents[1]


def test_measured_waypoints_match_original_image():
    result = load_waypoints(SHARE/'config/waypoints.yaml', SHARE/'maps/map.yaml')
    assert len(result['goals']) >= 3
    assert all('SUCCEEDED UUID' in goal['source'] for goal in result['goals'])


@pytest.mark.parametrize('field,value', [('validated', False), ('map_image_sha256', 'wrong')])
def test_unverified_or_wrong_map_is_rejected(tmp_path, field, value):
    data = yaml.safe_load((SHARE/'config/waypoints.yaml').read_text())
    data[field] = value
    path = tmp_path/'goals.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_waypoints(path, SHARE/'maps/map.yaml')


def test_nonfinite_goal_is_rejected(tmp_path):
    data = yaml.safe_load((SHARE/'config/waypoints.yaml').read_text())
    data['goals'][0]['yaw'] = float('nan')
    path = tmp_path/'goals.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_waypoints(path, SHARE/'maps/map.yaml')


def test_unknown_obstacle_and_rotated_map_boundaries_block_targets():
    grid = OccupancyGrid()
    grid.info.width = grid.info.height = 20
    grid.info.resolution = 0.1
    grid.info.origin.orientation.z = math.sin(math.pi/4)
    grid.info.origin.orientation.w = math.cos(math.pi/4)
    grid.data = [0]*400
    assert point_is_free(grid, -1.0, 1.0, 0.2)
    grid.data[10*20+10] = -1
    assert not point_is_free(grid, -1.0, 1.0, 0.2)
    grid.data[10*20+10] = 100
    assert not point_is_free(grid, -1.0, 1.0, 0.2)
    assert not point_is_free(grid, 0.1, 1.0, 0.2)


def test_yaw_quaternion_rotates_forward_vector():
    _, _, z, w = quaternion_from_yaw(math.pi/2)
    # Quaternion action on (1,0,0) must point along +Y, including a unit norm.
    assert math.isclose(z*z+w*w, 1.0)
    assert abs(1-2*z*z) < 1e-12
    assert math.isclose(2*w*z, 1.0)
