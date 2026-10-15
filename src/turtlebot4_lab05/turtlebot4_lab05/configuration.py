"""Validate measured waypoints and map identity before any action is sent."""
import hashlib
import math
from pathlib import Path
import yaml


def load_waypoints(filename, map_yaml):
    data = yaml.safe_load(Path(filename).expanduser().read_text())
    if not isinstance(data, dict) or data.get('validated') is not True:
        raise ValueError('Waypoint YAML chua duoc xac thuc: validated phai la true')
    if data.get('frame_id') != 'map':
        raise ValueError('Waypoint frame_id phai la map')
    config_path = Path(map_yaml).expanduser().resolve()
    map_data = yaml.safe_load(config_path.read_text())
    image = config_path.parent / map_data['image']
    digest = hashlib.sha256(image.read_bytes()).hexdigest()
    if data.get('map_image_sha256') != digest:
        raise ValueError('Waypoint khong thuoc anh ban do dang nap (SHA256 mismatch)')
    goals = data.get('goals', [])
    if len(goals) < 3:
        raise ValueError('Can it nhat 3 goal da do')
    names = set()
    for goal in goals:
        if not isinstance(goal, dict) or not goal.get('name') or not goal.get('source'):
            raise ValueError('Moi goal can name va source bang chung')
        if goal['name'] in names:
            raise ValueError('Goal name bi trung')
        names.add(goal['name'])
        for key in ('x', 'y', 'yaw'):
            value = goal.get(key)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f'Goal {goal["name"]}: {key} khong hop le')
    return data


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def point_is_free(grid, x, y, margin=0.25):
    """Require a known free disk, including origin rotation and map boundaries."""
    q = grid.info.origin.orientation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    dx, dy = x-grid.info.origin.position.x, y-grid.info.origin.position.y
    gx = (math.cos(yaw)*dx + math.sin(yaw)*dy) / grid.info.resolution
    gy = (-math.sin(yaw)*dx + math.cos(yaw)*dy) / grid.info.resolution
    radius = margin/grid.info.resolution
    for row in range(math.floor(gy-radius), math.ceil(gy+radius)+1):
        for col in range(math.floor(gx-radius), math.ceil(gx+radius)+1):
            if math.hypot(col+0.5-gx, row+0.5-gy) > radius + 0.71:
                continue
            if not (0 <= col < grid.info.width and 0 <= row < grid.info.height):
                return False
            value = grid.data[row*grid.info.width+col]
            if value < 0 or value >= 50:
                return False
    return True
