#!/usr/bin/env python3
"""Download the versioned official model and verify SHA256 before use."""
import argparse,hashlib,json,os
from pathlib import Path
import urllib.request


def main():
 p=argparse.ArgumentParser();p.add_argument('--cache',type=Path,default=Path(os.getenv('TB4_GESTURE_CACHE',str(Path.home()/'.cache/turtlebot4_gesture_control'))));a=p.parse_args()
 manifest=json.loads((Path(__file__).resolve().parents[1]/'config/model_manifest.json').read_text());a.cache.mkdir(parents=True,exist_ok=True)
 target=a.cache/'gesture_recognizer.task'
 if target.is_file() and hashlib.sha256(target.read_bytes()).hexdigest()==manifest['sha256']:
  print('Model already verified:',target);return
 temporary=target.with_suffix('.download')
 try:
  with urllib.request.urlopen(manifest['url'],timeout=60) as source,temporary.open('wb') as dest:
   while chunk:=source.read(1024*1024):dest.write(chunk)
  if hashlib.sha256(temporary.read_bytes()).hexdigest()!=manifest['sha256']:raise RuntimeError('Model hash mismatch')
  temporary.replace(target);print('Downloaded and verified:',target)
 finally:temporary.unlink(missing_ok=True)
if __name__=='__main__':main()
