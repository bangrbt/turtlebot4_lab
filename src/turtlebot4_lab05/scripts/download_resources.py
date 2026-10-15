#!/usr/bin/env python3
"""Download official model weights and licensed Fuel assets before the demo."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
from urllib.parse import quote, unquote
import requests
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, default=Path(os.getenv('TB4_LAB05_CACHE', str(Path.home()/'.cache/turtlebot4_lab05'))))
    parser.add_argument('--assets', type=Path, default=Path(__file__).resolve().parents[1]/'config/assets.yaml')
    args = parser.parse_args()
    cache = args.cache.expanduser().resolve()
    cache.mkdir(parents=True, exist_ok=True)
    weights = cache/'yolo11n.pt'
    weights_url = 'https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt'
    if not weights.is_file():
        temporary = weights.with_suffix('.part')
        with requests.get(weights_url, timeout=(15, 120), stream=True) as response:
            response.raise_for_status()
            with temporary.open('wb') as out:
                for block in response.iter_content(1024*1024):
                    out.write(block)
        temporary.replace(weights)
    lock_file = args.assets.parent/'resources_manifest.json'
    expected = json.loads(lock_file.read_text()) if lock_file.is_file() else {}
    digest = hashlib.sha256(weights.read_bytes()).hexdigest()
    if expected.get('weights', {}).get('sha256') not in (None, digest):
        raise RuntimeError('Weights SHA256 mismatch; refusing demo')
    manifest = dict(weights=dict(url=weights_url, sha256=hashlib.sha256(weights.read_bytes()).hexdigest()), assets={})
    config = yaml.safe_load(args.assets.read_text())
    for key, asset in config['assets'].items():
        url = 'https://fuel.gazebosim.org/1.0/'+asset['owner']+'/models/'+quote(asset['model'])
        response = requests.get(url, timeout=30)
        response.raise_for_status()
        metadata = response.json()
        version = asset.get('version', metadata['version'])
        destination = cache/'models'/key
        if not (destination/'model.sdf').is_file():
            subprocess.run(['ign', 'fuel', 'download', '-u', url+'/'+str(version)], check=True, timeout=180)
            candidates = []
            for root in [Path.home()/'.ignition/fuel', Path.home()/'.gz/fuel']:
                if root.exists():
                    candidates.extend(p for p in root.rglob('model.sdf')
                                      if p.parent.name == str(version) and unquote(p.parent.parent.name).lower() == asset['model'].lower())
            if len(candidates) != 1:
                raise RuntimeError(f'Cannot uniquely locate downloaded {asset["model"]}: {candidates}')
            shutil.copytree(candidates[0].parent, destination, dirs_exist_ok=True)
        sdf_hash = hashlib.sha256((destination/'model.sdf').read_bytes()).hexdigest()
        if asset.get('source_sdf_sha256') not in (None, sdf_hash):
            raise RuntimeError('Asset SDF SHA256 mismatch: '+key)
        manifest['assets'][key] = dict(url=url, version=version, license=metadata.get('license_name'),
                                       license_url=metadata.get('license_url'),
                                       source_sdf_sha256=hashlib.sha256((destination/'model.sdf').read_bytes()).hexdigest())
        print(key, manifest['assets'][key], flush=True)
    (cache/'resources_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print('Resources ready:', cache, flush=True)


if __name__ == '__main__':
    main()
