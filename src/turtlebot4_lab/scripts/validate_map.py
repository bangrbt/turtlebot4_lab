#!/usr/bin/env python3
"""Validate a saved raster map; never creates a map."""
import argparse
import json
import math
from pathlib import Path
import yaml
from PIL import Image


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('yaml', type=Path)
    args = p.parse_args()
    try:
        path = args.yaml.expanduser().resolve()
        data = yaml.safe_load(path.read_text())
        if not isinstance(data, dict):
            raise ValueError('YAML phai la mapping')
        image = path.parent / data['image']
        resolution = float(data['resolution'])
        origin = data['origin']
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('resolution khong hop le')
        if len(origin) != 3 or not all(math.isfinite(float(v)) for v in origin):
            raise ValueError('origin can [x, y, yaw] huu han')
        if not 0 <= float(data['free_thresh']) < float(data['occupied_thresh']) <= 1:
            raise ValueError('nguong free/occupied khong hop le')
        if data['negate'] not in (0, 1):
            raise ValueError('negate can 0 hoac 1')
        with Image.open(image) as im:
            im.load()
            if im.width < 1 or im.height < 1:
                raise ValueError('anh ban do rong')
            size = im.size
        print(json.dumps({'yaml': str(path), 'image': str(image), 'size_pixels': size,
                          'resolution': resolution, 'origin': origin,
                          'portable_image_path': not Path(data['image']).is_absolute()}, indent=2))
        return 0
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        p.exit(1, f'LOI BAN DO: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
