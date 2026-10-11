#!/usr/bin/env python3
"""Save the live /map through the official Nav2 map saver into a NEW output pair."""
import argparse
import subprocess
import sys
from pathlib import Path
import yaml


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True, type=Path, help='Absolute prefix, e.g. .../maps/map')
    args = p.parse_args()
    prefix = args.output.expanduser().resolve()
    paths = [Path(str(prefix) + '.yaml'), Path(str(prefix) + '.pgm')]
    if any(path.exists() for path in paths):
        p.exit(1, 'LOI: output da ton tai; chon prefix moi hoac sao luu ban do cu truoc.\n')
    prefix.parent.mkdir(parents=True, exist_ok=True)
    command = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(prefix),
               '--fmt', 'pgm', '--ros-args', '-p', 'use_sim_time:=true', '-p',
               'map_subscribe_transient_local:=true', '-p', 'save_map_timeout:=15.0']
    print('Saving LIVE /map:', ' '.join(command), flush=True)
    try:
        result = subprocess.run(command, timeout=45, check=False)
    except subprocess.TimeoutExpired:
        p.exit(1, 'LOI: map saver timeout. Kiem tra /clock, /map va SLAM.\n')
    if result.returncode or not all(path.is_file() for path in paths):
        p.exit(1, 'LOI: chua luu duoc ca YAML/PGM. Kiem tra log map_saver_cli.\n')
    data = yaml.safe_load(paths[0].read_text())
    saved_image = (paths[0].parent / data['image']).resolve()
    if saved_image != paths[1]:
        p.exit(1, f'LOI: map saver tro anh ngoai output mong doi: {saved_image}\n')
    data['image'] = paths[1].name
    paths[0].write_text(yaml.safe_dump(data, sort_keys=False))
    return subprocess.call([sys.executable, str(Path(__file__).with_name('validate_map.py')), str(paths[0])])


if __name__ == '__main__':
    raise SystemExit(main())
