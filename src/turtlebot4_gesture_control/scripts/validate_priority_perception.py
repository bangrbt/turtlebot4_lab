#!/usr/bin/env python3
"""Real MediaPipe on official fixtures / synthetic motion, not a webcam accuracy benchmark."""
import argparse
import json
import os
from pathlib import Path
import time
import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from turtlebot4_gesture_control.gestures import RecognitionPolicy


def hands(result):
    return [(sorted([(c.category_name,float(c.score)) for c in categories],key=lambda c:c[1],reverse=True),
             [(p.x,p.y,p.z) for p in landmarks])
            for categories,landmarks in zip(result.gestures,result.hand_landmarks)]


def image(frame):
    return mp.Image(image_format=mp.ImageFormat.SRGB,data=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache',type=Path,default=Path(os.environ.get('TB4_GESTURE_CACHE','.cache_gesture')))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();samples=args.cache/'official_samples';model_path=args.cache/'gesture_recognizer.task'
    result=dict(scope=__doc__,fixtures=[],wave={})
    try:
        options=vision.GestureRecognizerOptions(base_options=python.BaseOptions(model_asset_path=str(model_path)),
                                                num_hands=1,min_hand_detection_confidence=.55,
                                                min_hand_presence_confidence=.50,min_tracking_confidence=.50)
        with vision.GestureRecognizer.create_from_options(options) as model:
            for name,expected in [('fist.jpg','Closed_Fist'),('thumb_up.jpg','Thumb_Up'),('pointing_up.jpg','Pointing_Up'),('victory.jpg','Victory')]:
                source=cv2.imread(str(samples/name))
                if source is None:raise RuntimeError('Run validate_model_samples.py once to cache official images')
                h,w=source.shape[:2]
                for degrees in [-25.,0.,25.]:
                    matrix=cv2.getRotationMatrix2D((w/2,h/2),degrees,1.)
                    rotated=cv2.warpAffine(source,matrix,(w,h),borderMode=cv2.BORDER_CONSTANT,borderValue=(255,255,255))
                    raw=hands(model.recognize(image(rotated)))
                    assert len(raw)==1,(name,degrees,'missing hand')
                    policy=RecognitionPolicy(confidence=.55,confirm_confidence=.60,margin=.08,frames=3,hold=.20)
                    first=policy.decide(raw,0.);second=policy.decide(raw,.1)
                    cancelled=second['label']=='Closed_Fist'
                    assert cancelled==(expected=='Closed_Fist'),(name,degrees,raw[0][0],second)
                    if expected=='Closed_Fist':
                        # Classifier-dropout reproduction: geometry must still cancel.
                        fallback=RecognitionPolicy()
                        dropped=[([('None',.9)],raw[0][1])]
                        fallback.decide(dropped,0.)
                        assert fallback.decide(dropped,.1)['label']=='Closed_Fist'
                    result['fixtures'].append(dict(image=name,rotation_degrees=degrees,raw_label=raw[0][0][0][0],
                        raw_confidence=raw[0][0][0][1],cancelled=cancelled,
                        cancel_frames=(1 if first['label']=='Closed_Fist' else 2) if cancelled else None))
        # Move an official palm image over a canvas; the video model really tracks each frame.
        palm=cv2.imread(str(samples/'right_hands.jpg'))[:,360:]
        palm=cv2.resize(palm,(210,240))
        options.running_mode=vision.RunningMode.VIDEO
        policy=RecognitionPolicy(confidence=.55,confirm_confidence=.60,margin=.08,frames=3,hold=.20)
        rows=[];began=time.monotonic()
        with vision.GestureRecognizer.create_from_options(options) as model:
            for i,shift in enumerate([0,35,70,35,0,-35,-70,-35,0,35,70,35]):
                canvas=np.full((480,640,3),235,dtype=np.uint8)
                x=215+shift;canvas[120:360,x:x+210]=palm
                raw=hands(model.recognize_for_video(image(canvas),1000+i*100))
                d=policy.decide(raw,i*.1)
                rows.append(dict(t=i*.1,label=d['label'],reason=d['reason'],raw_label=d['raw_label'],raw_confidence=d['raw_confidence']))
        first_wave=next((r['t'] for r in rows if r['label']=='Wave'),None)
        assert first_wave is not None and first_wave<=.8,rows
        result['wave']=dict(first_wave_seconds=first_wave,frames=rows,wall_seconds=time.monotonic()-began)
        result['passed']=True
    except Exception as exc:
        import traceback
        traceback.print_exc();result.update(passed=False,error=repr(exc))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
