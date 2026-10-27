"""Conservative scene checks against saved occupancy and actual maze wall boxes."""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml


def _pose(element):
    node = element.find('pose')
    values = [float(v) for v in node.text.split()] if node is not None else [0.0]*6
    if abs(values[3])+abs(values[4]) > 1e-6:
        raise ValueError('Scene validator requires ground-plane wall transforms')
    return values[0], values[1], values[5]


def _compose(a, b):
    x, y, yaw = a
    u, v, theta = b
    return x+math.cos(yaw)*u-math.sin(yaw)*v, y+math.sin(yaw)*u+math.cos(yaw)*v, yaw+theta


def maze_wall_boxes(world_file):
    boxes = []
    root = ET.parse(world_file).getroot()
    for model in root.findall('./world/model'):
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                size = collision.find('geometry/box/size')
                if size is None:
                    continue  # Ground plane is not a wall.
                dx, dy, dz = [float(v) for v in size.text.split()]
                if dz < 0.2:
                    continue
                x, y, yaw = _compose(_compose(_pose(model), _pose(link)), _pose(collision))
                boxes.append((x, y, yaw, dx/2, dy/2))
    if not boxes:
        raise ValueError('No wall boxes found; cannot validate this world')
    return boxes


def wall_distance(x, y, boxes):
    distances = []
    for bx, by, yaw, hx, hy in boxes:
        u = math.cos(yaw)*(x-bx)+math.sin(yaw)*(y-by)
        v = -math.sin(yaw)*(x-bx)+math.cos(yaw)*(y-by)
        distances.append(math.hypot(max(abs(u)-hx, 0), max(abs(v)-hy, 0)))
    return min(distances)


def validate_scene(config, map_file, world_file):
    if config.get('validated') is not True or config.get('world_map_alignment_verified') is not True:
        raise ValueError('Scene needs explicit validation and verified world/map alignment')
    meta = yaml.safe_load(Path(map_file).read_text())
    image = Path(map_file).parent/meta['image']
    digest = hashlib.sha256(image.read_bytes()).hexdigest()
    if config.get('map_image_sha256') != digest:
        raise ValueError('Scene does not belong to this map image')
    pixels = np.asarray(Image.open(image).convert('L'), dtype=float)/255
    probability = pixels if meta.get('negate', 0) else 1-pixels
    known_free = probability < meta['free_thresh']
    resolution = float(meta['resolution'])
    distances = distance_transform_edt(np.pad(known_free, 1))[1:-1, 1:-1]*resolution
    boxes = maze_wall_boxes(world_file)
    origin_x, origin_y, yaw = meta['origin']
    minimum_gap = float(config.get('minimum_wall_gap', 0.35))
    minimum_pair_gap = float(config.get('minimum_object_gap', 0.60))
    rows = []
    names = set()
    for item in config['objects']:
        name = item['name']
        if not name.startswith('lab05_') or name in names:
            raise ValueError('Unique lab05_ object names required')
        names.add(name)
        x, y, _, roll, pitch, _ = item['pose']
        if roll or pitch:
            raise ValueError('Objects must stand on flat ground')
        dx, dy, dz = item['collision_size']
        radius = max(float(item.get('footprint_radius', 0)), math.hypot(dx, dy)/2)
        if min(dx, dy, dz, radius) <= 0 or not all(math.isfinite(v) for v in item['pose']+[radius]):
            raise ValueError('Invalid object size/pose: '+name)
        u = math.cos(yaw)*(x-origin_x)+math.sin(yaw)*(y-origin_y)
        v = -math.sin(yaw)*(x-origin_x)+math.cos(yaw)*(y-origin_y)
        col = math.floor(u/resolution)
        row = pixels.shape[0]-1-math.floor(v/resolution)
        if not (0 <= row < pixels.shape[0] and 0 <= col < pixels.shape[1]):
            raise ValueError('Object outside map: '+name)
        map_gap = distances[row, col]-radius-resolution*math.sqrt(2)/2
        world_gap = wall_distance(x, y, boxes)-radius
        if min(map_gap, world_gap) < minimum_gap:
            raise ValueError(f'{name}: insufficient body-to-wall/unknown gap: map={map_gap:.3f}, world={world_gap:.3f}, required={minimum_gap:.3f} m')
        for previous in rows:
            gap = math.hypot(x-previous['x'], y-previous['y'])-radius-previous['radius']
            if gap < minimum_pair_gap:
                raise ValueError(f'{name} too close to {previous["name"]}: gap={gap:.3f} m')
        rows.append(dict(name=name, asset=item['asset'], x=x, y=y, radius=radius,
                         map_body_gap_m=float(map_gap), world_wall_body_gap_m=world_gap))
    return dict(passed=True, map_image_sha256=digest,
                world_sdf_sha256=hashlib.sha256(Path(world_file).read_bytes()).hexdigest(),
                minimum_wall_gap_m=minimum_gap, minimum_object_gap_m=minimum_pair_gap,
                objects=rows, scope='Conservative enclosing XY disks; known-free map including unknown and actual maze wall boxes; not a navigation/collision guarantee.')
