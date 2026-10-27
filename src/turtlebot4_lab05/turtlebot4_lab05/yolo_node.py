"""Latest-frame RGB inference in a worker; no velocity or navigation publisher."""
import json
import os
import signal
import threading
import time
from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock as RclpyClock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from .detection_filter import DetectionConfirmation


class YoloNode(Node):
    def __init__(self):
        super().__init__('lab05_yolo')
        defaults = dict(camera_topic='/oakd/rgb/preview/image_raw',
                        annotated_topic='/yolo/annotated_image', weights='yolo11s.pt',
                        device='0', cpu_fallback=True, confidence=0.30, image_size=640,
                        inference_rate=3.0, show_window=True, camera_timeout=4.0,
                        log_interval=2.0, classes=['person', 'bicycle', 'chair', 'potted plant', 'horse', 'fire hydrant'],
                        person_confidence=0.55, plant_confidence=0.15, min_box_height=35,
                        person_aspect_min=1.1, person_aspect_max=4.5,
                        log_file='', torch_threads=1, nms_iou=0.45, confirmation_frames=2,
                        confirmation_iou=0.25, confirmation_timeout=1.5)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.params = {key: self.get_parameter(key).value for key in defaults}
        if not 0 < self.params['confidence'] <= 1 or self.params['inference_rate'] <= 0:
            raise ValueError('confidence in (0,1], inference_rate > 0 required')
        if self.params['image_size'] < 32 or self.params['camera_timeout'] <= 0:
            raise ValueError('Invalid image_size or camera_timeout')
        weights = Path(os.path.expandvars(self.params['weights'])).expanduser()
        if not weights.is_absolute():
            cache_dir = Path(os.getenv('TB4_LAB05_CACHE')) if os.getenv('TB4_LAB05_CACHE') else None
            if not cache_dir:
                cache_dir = Path.cwd()/'.cache_lab05' if (Path.cwd()/'.cache_lab05').is_dir() else Path.home()/'.cache/turtlebot4_lab05'
            weights = cache_dir/weights
        if not weights.is_file():
            raise FileNotFoundError(f'Weights missing: {weights}. Run download_resources.py first; no download during demo.')
        import torch
        from ultralytics import YOLO
        self.torch = torch
        torch.set_num_threads(self.params['torch_threads'])
        requested = str(self.params['device']).lower()
        if requested == 'cpu':
            self.device = 'cpu'
        elif torch.cuda.is_available():
            index = int(requested.replace('cuda:', ''))
            if index >= torch.cuda.device_count():
                raise ValueError(f'CUDA device {index} absent')
            self.device = f'cuda:{index}'
        elif self.params['cpu_fallback']:
            self.device = 'cpu'
            self.get_logger().warning('CUDA unavailable: explicit configured CPU fallback')
        else:
            raise RuntimeError('CUDA unavailable and cpu_fallback=false')
        if not 0 < self.params['nms_iou'] <= 1:
            raise ValueError('nms_iou in (0,1] required')
        self.confirmation = DetectionConfirmation(self.params['confirmation_frames'],
            self.params['confirmation_iou'], self.params['confirmation_timeout'])
        self.model = YOLO(str(weights))
        self.class_ids = [index for index, name in self.model.names.items() if name in self.params['classes']]
        missing = set(self.params['classes'])-set(self.model.names.values())
        if missing or not self.class_ids:
            raise ValueError(f'Unsupported class names: {sorted(missing)}')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.wakeup = threading.Event()
        self.stop_event = threading.Event()
        self.latest = None
        self.sequence = 0
        self.last_received = 0.0
        self.preview = None
        self.preview_created = False
        self.last_success = 0.0
        self.last_inference_ms = None
        self.frames = 0
        self.actual_device = 'not_inferred'
        self.started = time.monotonic()
        self.last_log = 0.0
        self.last_status = None
        self.last_warning = 0.0
        self.detections = []
        self.error = None
        self.fatal = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.image_publisher = self.create_publisher(Image, self.params['annotated_topic'], qos)
        self.status_publisher = self.create_publisher(String, '/yolo/status', tl)
        self.detection_publisher = self.create_publisher(String, '/yolo/detections', 10)
        self.subscription = self.create_subscription(Image, self.params['camera_topic'], self.on_image, qos)
        if self.params['show_window'] and not os.getenv('DISPLAY'):
            self.get_logger().warning('DISPLAY absent: headless mode')
            self.params['show_window'] = False
        self.wall_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self.tick, clock=self.wall_clock)
        self.worker = threading.Thread(target=self.inference, name='lab05-yolo-inference', daemon=True)
        self.log('model_loaded', model=str(weights), requested_device=requested, selected_device=self.device,
                 torch=torch.__version__, cuda_available=torch.cuda.is_available(),
                 gpu=torch.cuda.get_device_name(int(self.device.split(':')[1])) if self.device != 'cpu' else None,
                 labels=self.model.names, filter=self.params['classes'])
        self.worker.start()

    def log(self, event, **values):
        record = dict(event=event, wall_time_unix=time.time(), **values)
        line = json.dumps(record, ensure_ascii=False)
        self.get_logger().info(line)
        if self.params['log_file']:
            path = Path(self.params['log_file']).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('a') as stream:
                stream.write(line+'\n')

    def on_image(self, msg):
        # O(1) callback: replace old work. No inference or image conversion here.
        with self.lock:
            self.latest = msg
            self.sequence += 1
            self.last_received = time.monotonic()
        self.wakeup.set()

    def inference(self):
        last_sequence = -1
        next_at = 0.0
        errors = 0
        while not self.stop_event.is_set():
            self.wakeup.wait(timeout=0.2)
            self.wakeup.clear()
            delay = max(0.0, next_at-time.monotonic())
            if self.stop_event.wait(delay):
                break
            with self.lock:
                msg, sequence = self.latest, self.sequence
                received = self.last_received
            if msg is None or sequence == last_sequence:
                continue
            last_sequence = sequence
            if time.monotonic()-received > self.params['camera_timeout']:
                continue
            began = time.monotonic()
            try:
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                predict_conf = min(self.params['confidence'], self.params.get('plant_confidence', 0.15))
                result = self.model.predict(source=image, device=self.device, imgsz=self.params['image_size'],
                                            conf=predict_conf, iou=self.params['nms_iou'], classes=self.class_ids,
                                            verbose=False)[0]
                if self.device != 'cpu':
                    self.torch.cuda.synchronize()
                elapsed_ms = (time.monotonic()-began)*1000
                self.actual_device = str(next(self.model.model.parameters()).device)
                detections = []
                keep_raw_indices = []
                if result.boxes is not None:
                    for raw_idx, box in enumerate(result.boxes):
                        index = int(box.cls.item())
                        label = self.model.names[index]
                        conf = float(box.conf.item())
                        xyxy = [float(v) for v in box.xyxy[0].tolist()]
                        w = max(0.0, xyxy[2] - xyxy[0])
                        h = max(0.0, xyxy[3] - xyxy[1])
                        # Filter false-positive person detections on walls, corners, or distant noise
                        if label == 'person':
                            if conf < self.params.get('person_confidence', 0.55):
                                continue
                            if h < self.params.get('min_box_height', 35):
                                continue
                            aspect = h / max(w, 1.0)
                            if aspect < self.params.get('person_aspect_min', 1.1) or aspect > self.params.get('person_aspect_max', 4.5):
                                continue
                        elif label == 'potted plant':
                            if conf < self.params.get('plant_confidence', 0.15):
                                continue
                        else:
                            if conf < self.params['confidence']:
                                continue
                        detections.append(dict(class_id=index, label=label,
                                               confidence=conf, xyxy=xyxy))
                        keep_raw_indices.append(raw_idx)
                raw_detection_count = len(detections)
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                keep = self.confirmation.update(detections, stamp, time.monotonic())
                detections = [detections[i] for i in keep]
                annotated_indices = [keep_raw_indices[i] for i in keep]
                annotated = result[annotated_indices].plot() if annotated_indices else image.copy()
                message = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                message.header = msg.header
                self.image_publisher.publish(message)
                payload = dict(stamp=dict(sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nanosec),
                               frame_id=msg.header.frame_id, detections=detections, device=self.device,
                               inference_ms=elapsed_ms, input_sequence=sequence, actual_model_device=self.actual_device,
                               raw_detection_count=raw_detection_count, confirmation_frames=self.params['confirmation_frames'])
                self.detection_publisher.publish(String(data=json.dumps(payload)))
                with self.lock:
                    self.preview = annotated if self.params['show_window'] else None
                    self.last_success = time.monotonic()
                    self.last_inference_ms = elapsed_ms
                    self.detections = detections
                    self.error = None
                    self.frames += 1
                errors = 0
                if time.monotonic()-self.last_log >= self.params['log_interval']:
                    self.last_log = time.monotonic()
                    self.log('inference', **payload, no_detection=not detections)
            except Exception as exc:
                self.confirmation.reset()
                errors += 1
                self.error = str(exc)
                self.log('inference_error', error=str(exc), consecutive_errors=errors)
                if self.device != 'cpu' and self.params['cpu_fallback'] and ('cuda' in str(exc).lower() or 'memory' in str(exc).lower()):
                    self.device = 'cpu'
                    self.model.to('cpu')
                    self.torch.cuda.empty_cache()
                    self.log('cpu_fallback', reason=str(exc))
                elif errors >= 3:
                    self.fatal = True
                    break
            next_at = time.monotonic()+1.0/self.params['inference_rate']

    def tick(self):
        now = time.monotonic()
        with self.lock:
            camera_age = now-self.last_received if self.last_received else None
            success_age = now-self.last_success if self.last_success else None
            preview = self.preview
            state = 'error' if self.fatal else ('waiting_camera' if camera_age is None or camera_age > self.params['camera_timeout'] else
                                              ('ready' if success_age is not None and success_age < self.params['camera_timeout'] else 'warming_up'))
            status = dict(state=state, device=self.device, cuda=self.actual_device.startswith('cuda'), actual_model_device=self.actual_device, frames=self.frames,
                          camera_topic=self.params['camera_topic'], camera_age_wall_s=camera_age,
                          last_success_age_wall_s=success_age, inference_ms=self.last_inference_ms,
                          average_fps=self.frames/max(1.0, now-self.started), error=self.error)
        if state != self.last_status or now-self.last_warning >= 1:
            self.status_publisher.publish(String(data=json.dumps(status)))
            if state != self.last_status:
                self.log('state_changed', **status)
            self.last_status = state
            self.last_warning = now
        if self.params['show_window'] and preview is not None:
            try:
                if not self.preview_created:
                    cv2.namedWindow('TurtleBot 4 Lab05 - YOLO RGB', cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
                    cv2.resizeWindow('TurtleBot 4 Lab05 - YOLO RGB', 800, 600)
                    self.preview_created = True
                # Enlarge only the local preview; keep the published image/header untouched.
                height = max(1, round(800*preview.shape[0]/preview.shape[1]))
                cv2.imshow('TurtleBot 4 Lab05 - YOLO RGB',
                           cv2.resize(preview, (800, height), interpolation=cv2.INTER_LINEAR))
                if cv2.waitKey(1) & 0xff in (27, ord('q')):
                    self.params['show_window'] = False
                    cv2.destroyAllWindows()
            except cv2.error as exc:
                self.params['show_window'] = False
                self.log('preview_disabled', reason=str(exc))

    def close(self):
        self.stop_event.set()
        self.wakeup.set()
        self.worker.join(timeout=8)
        if self.worker.is_alive():
            self.get_logger().error('Inference thread did not finish before shutdown timeout')
        if self.params['show_window']:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = None
    stopped = False
    def stop(signum, frame):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        node = YoloNode()
        while rclpy.ok() and not stopped and not node.fatal:
            rclpy.spin_once(node, timeout_sec=0.1)
        return 1 if node.fatal else 0
    except Exception as exc:
        print(f'LOI YOLO: {exc}', flush=True)
        return 1
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
