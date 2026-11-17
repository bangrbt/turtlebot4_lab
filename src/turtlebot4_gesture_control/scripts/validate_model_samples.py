#!/usr/bin/env python3
"""Five official sample images, raw model smoke test; not an accuracy benchmark."""
import argparse,hashlib,json,os,urllib.request
from pathlib import Path
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from turtlebot4_gesture_control.gestures import RecognitionPolicy


def main():
 p=argparse.ArgumentParser();p.add_argument('--cache',type=Path,default=Path(os.getenv('TB4_GESTURE_CACHE',str(Path.home()/'.cache/turtlebot4_gesture_control'))));p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 examples=a.cache/'official_samples';examples.mkdir(parents=True,exist_ok=True)
 tests=[('pointing_up.jpg','Pointing_Up',None),('victory.jpg','Victory',None),('thumb_up.jpg','Thumb_Up',None),('fist.jpg','Closed_Fist',None),('right_hands.jpg','Open_Palm',(360,0,720,382))]
 rows=[]
 opts=vision.GestureRecognizerOptions(base_options=python.BaseOptions(model_asset_path=str(a.cache/'gesture_recognizer.task'),delegate=python.BaseOptions.Delegate.CPU),num_hands=2)
 with vision.GestureRecognizer.create_from_options(opts) as model:
  for filename,expected,crop in tests:
   url='https://storage.googleapis.com/mediapipe-assets/'+filename;f=examples/filename
   if not f.is_file():
    with urllib.request.urlopen(url,timeout=30) as source:f.write_bytes(source.read())
   frame=cv2.imread(str(f))
   if crop:x0,y0,x1,y1=crop;frame=frame[y0:y1,x0:x1]
   result=model.recognize(mp.Image(image_format=mp.ImageFormat.SRGB,data=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)))
   hands=[]
   for classes,landmarks in zip(result.gestures,result.hand_landmarks):
    hands.append((sorted([(c.category_name,float(c.score)) for c in classes],key=lambda v:v[1],reverse=True),[(p.x,p.y,p.z) for p in landmarks]))
   raw=hands[0][0][0] if len(hands)==1 else ('None',0.)
   policy=RecognitionPolicy();decision=policy.decide(hands,1.)
   rows.append(dict(source=url,source_sha256=hashlib.sha256(f.read_bytes()).hexdigest(),crop=crop,expected=expected,raw_label=raw[0],confidence=raw[1],raw_label_matches=raw[0]==expected,control_policy=decision))
 result=dict(scope='Single official sample per class; raw classification smoke test, not held-out/live accuracy. Low confidence is intentionally rejected by control policy.',samples=rows,raw_matches=sum(r['raw_label_matches'] for r in rows),passed=all(r['raw_label_matches'] for r in rows))
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
 return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
