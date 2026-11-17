"""Latest-frame V4L2 capture. A blocked read cannot keep the robot heartbeat alive."""
import threading
import time
import zlib
import cv2


class Webcam:
    def __init__(self,index=0,width=640,height=480,fps=15):
        self.index,self.width,self.height,self.fps=index,width,height,fps
        self.lock=threading.Lock();self.stop_event=threading.Event()
        self.latest=None;self.sequence=0;self.error=None
        self.thread=threading.Thread(target=self.run,name='webcam-v4l2',daemon=True)
        self.thread.start()

    def run(self):
        cap=None
        try:
            cap=cv2.VideoCapture(self.index,cv2.CAP_V4L2)
            if not cap.isOpened():raise RuntimeError(f'Cannot open /dev/video{self.index}')
            cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,self.width);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,self.height)
            cap.set(cv2.CAP_PROP_FPS,self.fps);cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
            checksum,last_change=None,time.monotonic()
            while not self.stop_event.is_set():
                ok,frame=cap.read()
                if not ok:raise RuntimeError('Webcam read failed/disconnected')
                mono,unix=time.monotonic(),time.time()
                crc=zlib.crc32(frame)
                if crc!=checksum:checksum,last_change=crc,mono
                if mono-last_change>1.:
                    raise RuntimeError('Webcam returned identical frozen frames for >1 second')
                with self.lock:
                    self.sequence+=1;self.latest=(self.sequence,frame,mono,unix)
        except Exception as exc:
            self.error=str(exc)
        finally:
            if cap is not None:cap.release()

    def read(self):
        with self.lock:return self.latest

    def close(self):
        self.stop_event.set();self.thread.join(timeout=1.)
