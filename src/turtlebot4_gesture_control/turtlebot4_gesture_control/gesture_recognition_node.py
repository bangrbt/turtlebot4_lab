"""OpenCV webcam -> MediaPipe Tasks Gesture Recognizer -> fresh JSON String."""
import hashlib
import json
import os
from pathlib import Path
import signal
import time
import uuid
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from .camera import Webcam
from .gestures import Observation,RecognitionPolicy

MODEL_SHA256='97952348cf6a6a4915c2ea1496b4b37ebabc50cbbf80571435643c455f2b0482'
CONNECTIONS=[(0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),(5,9),(9,10),(10,11),(11,12),(9,13),(13,14),(14,15),(15,16),(13,17),(0,17),(17,18),(18,19),(19,20)]


class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')
        cache=Path(os.getenv('TB4_GESTURE_CACHE',str(Path.home()/'.cache/turtlebot4_gesture_control')))
        defaults=dict(camera_index=0,width=640,height=480,camera_fps=15,show_window=True,mirror=True,
                      inference_enabled=True,model_path=str(cache/'gesture_recognizer.task'),model_sha256=MODEL_SHA256,
                      confidence=.55,confirmation_confidence=.60,class_margin=.08,stable_frames=3,stable_seconds=.20,
                      fist_confidence=.50,max_hands=1,camera_timeout=.45,gesture_topic='/detected_gesture')
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}
        self.policy=RecognitionPolicy(self.p['confidence'],self.p['class_margin'],self.p['confirmation_confidence'],self.p['stable_frames'],self.p['stable_seconds'],self.p['fist_confidence'])
        self.publisher=self.create_publisher(String,self.p['gesture_topic'],1)
        self.status_publisher=self.create_publisher(String,'/gesture/recognizer_status',10)
        self.session=uuid.uuid4().hex;self.sequence=0;self.last_frame=-1;self.last_publish=0.;self.last_ms=-1
        self.behavior={};self.frames=0;self.started=time.monotonic();self.mp=None;self.recognizer=None
        self.subscription=self.create_subscription(String,'/gesture/behavior_status',self.on_behavior,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self.p['inference_enabled']:
            path=Path(os.path.expandvars(self.p['model_path'])).expanduser()
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=self.p['model_sha256']:
                raise RuntimeError('Missing/wrong model; run scripts/setup_environment.bash before starting')
            import mediapipe as mp
            from mediapipe.tasks import python
            from mediapipe.tasks.python import vision
            self.mp=mp
            options=vision.GestureRecognizerOptions(base_options=python.BaseOptions(model_asset_path=str(path),delegate=python.BaseOptions.Delegate.CPU),
                running_mode=vision.RunningMode.VIDEO,num_hands=self.p['max_hands'],min_hand_detection_confidence=.55,
                min_hand_presence_confidence=.50,min_tracking_confidence=.50,
                canned_gesture_classifier_options=python.components.processors.ClassifierOptions(max_results=-1,score_threshold=0.))
            self.recognizer=vision.GestureRecognizer.create_from_options(options)
        if self.p['show_window'] and not os.getenv('DISPLAY'):self.p['show_window']=False
        self.camera=Webcam(self.p['camera_index'],self.p['width'],self.p['height'],self.p['camera_fps'])
        self.exit_requested=False;self.window_created=False

    def on_behavior(self,msg):
        try:self.behavior=json.loads(msg.data)
        except (ValueError,TypeError):pass

    def publish(self,decision,captured_unix):
        self.sequence+=1
        obs=Observation(session_id=self.session,sequence=self.sequence,captured_unix=captured_unix,**decision)
        self.publisher.publish(String(data=obs.dumps()));self.last_publish=time.monotonic()
        return obs

    def step(self):
        data=self.camera.read();now=time.monotonic()
        if data is None or now-data[2]>self.p['camera_timeout'] or self.camera.error:
            self.policy.reset()
            if now-self.last_publish>.10:
                self.publish(self.policy.unknown('camera_error' if self.camera.error else 'camera_timeout'),time.time())
            if self.camera.error:raise RuntimeError(self.camera.error)
            if now-self.started>8 and data is None:raise RuntimeError('No webcam frames after startup timeout')
            return
        sequence,frame,captured_mono,captured_unix=data
        if sequence==self.last_frame:return
        self.last_frame=sequence
        if self.p['mirror']:frame=cv2.flip(frame,1)
        began=time.monotonic();hands=[]
        if self.recognizer:
            rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
            image=self.mp.Image(image_format=self.mp.ImageFormat.SRGB,data=rgb)
            ms=max(self.last_ms+1,int(captured_mono*1000));self.last_ms=ms
            result=self.recognizer.recognize_for_video(image,ms)
            for categories,landmarks in zip(result.gestures,result.hand_landmarks):
                scores=sorted([(c.category_name,float(c.score)) for c in categories],key=lambda x:x[1],reverse=True)
                points=[(p.x,p.y,p.z) for p in landmarks]
                hands.append((scores,points))
            decision=self.policy.decide(hands,captured_mono)
        else:decision=self.policy.unknown('preview_only')
        # A slow inference must never turn an old frame into fresh control input.
        if time.monotonic()-captured_mono>self.p['camera_timeout']:
            self.policy.reset();decision=self.policy.unknown('inference_stale')
        obs=self.publish(decision,captured_unix);self.frames+=1
        elapsed=(time.monotonic()-began)*1000
        if obs.label in {'Wave','Closed_Fist'} or self.frames%15==1:
            self.status_publisher.publish(String(data=json.dumps(dict(frames=self.frames,label=obs.label,
                confidence=obs.confidence,raw_label=obs.raw_label,raw_confidence=obs.raw_confidence,
                stable=obs.stable,valid_control=obs.valid_control,inference_ms=elapsed,
                average_fps=self.frames/max(.001,time.monotonic()-self.started),reason=obs.reason))))
        if self.p['show_window']:self.show(frame,hands,obs,elapsed)

    def show(self,frame,hands,obs,elapsed):
        h,w=frame.shape[:2]
        for _,points in hands:
            for a,b in CONNECTIONS:cv2.line(frame,(int(points[a][0]*w),int(points[a][1]*h)),(int(points[b][0]*w),int(points[b][1]*h)),(90,220,80),2)
            for x,y,_ in points:cv2.circle(frame,(int(x*w),int(y*h)),3,(80,190,255),-1)
        cv2.rectangle(frame,(0,0),(w,110),(25,25,25),-1)
        accepted = obs.label + (' [landmarks]' if obs.reason in {'fist_landmarks_cancel','wave_landmarks'} else '')
        cur_state = self.behavior.get("state","IDLE")
        cur_sel = self.behavior.get("selected") or "-"
        if cur_state == "SELECTED":
            state_text = f"STATE: SELECTED [{cur_sel}] -> THUMB UP ONCE TO START"
        elif cur_state == "RUNNING":
            state_text = f"STATE: RUNNING [{cur_sel}] | HANDS FREE - finishes one round"
        else:
            state_text = f"STATE: IDLE | Selected: {cur_sel} | {'STABLE' if obs.stable else obs.reason}"
        lines=[f'Detected: {obs.raw_label} {obs.raw_confidence:.2f} | accepted: {accepted}',
               state_text,
               self.behavior.get('reason','Wave to greet; select 1/2 fingers then Thumb Up'),
               f'Open Palm: STOP | Fist: CANCEL | inference {elapsed:.0f} ms']
        for i,line in enumerate(lines):cv2.putText(frame,line,(10,23+25*i),cv2.FONT_HERSHEY_SIMPLEX,.49,(240,240,240),1,cv2.LINE_AA)
        if not self.window_created:
            cv2.namedWindow('TurtleBot 4 - Hand Gesture Control',cv2.WINDOW_NORMAL)
            cv2.resizeWindow('TurtleBot 4 - Hand Gesture Control',800,600);self.window_created=True
        cv2.imshow('TurtleBot 4 - Hand Gesture Control',frame)
        key=cv2.waitKey(1)&255
        if key in (27,ord('q')) or cv2.getWindowProperty('TurtleBot 4 - Hand Gesture Control',cv2.WND_PROP_VISIBLE)<1:self.exit_requested=True

    def close(self):
        self.publish(self.policy.unknown('recognizer_shutdown'),time.time())
        self.camera.close()
        if self.recognizer:self.recognizer.close()
        if self.p['show_window']:cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args);node=None;stopped=False
    def stop(signum,frame):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        node=GestureRecognitionNode()
        while rclpy.ok() and not stopped and not node.exit_requested:
            rclpy.spin_once(node,timeout_sec=.005);node.step()
        return 0
    except Exception as exc:
        print(f'Gesture recognizer error: {exc}',flush=True);return 1
    finally:
        if node:node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
