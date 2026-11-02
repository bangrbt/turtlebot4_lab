"""Confirm fresh model predictions across adjacent frames; never invent labels."""
import math


def box_iou(a, b):
    intersection = max(0.0, min(a[2], b[2])-max(a[0], b[0])) * max(0.0, min(a[3], b[3])-max(a[1], b[1]))
    area_a = max(0.0, a[2]-a[0])*max(0.0, a[3]-a[1])
    area_b = max(0.0, b[2]-b[0])*max(0.0, b[3]-b[1])
    return intersection/max(area_a+area_b-intersection, 1e-9)


class DetectionConfirmation:
    def __init__(self, frames=2, iou=0.25, timeout=1.5):
        if not isinstance(frames, int) or isinstance(frames, bool) or frames < 1:
            raise ValueError('confirmation_frames must be a positive integer')
        if not 0 < iou <= 1 or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Invalid confirmation IoU/timeout')
        self.frames, self.iou, self.timeout = frames, iou, timeout
        self.reset()

    def reset(self):
        self.previous = []
        self.last_stamp = None
        self.last_wall_time = None

    def update(self, detections, stamp, wall_time=None):
        now = stamp if wall_time is None else wall_time
        # ROS image stamps prevent one frozen/repeated image from confirming itself.
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.reset()
            return []
        if self.last_stamp is not None and now-self.last_wall_time > self.timeout:
            self.reset()
        used, current, keep = set(), [], []
        for index in sorted(range(len(detections)), key=lambda i: detections[i]['confidence'], reverse=True):
            detection = detections[index]
            matches = [(box_iou(detection['xyxy'], old['box']), i) for i,old in enumerate(self.previous)
                       if i not in used and old['class_id'] == detection['class_id']]
            score, match = max(matches, default=(0.0, -1))
            hits = 1
            if score >= self.iou:
                used.add(match)
                hits = self.previous[match]['hits']+1
            current.append(dict(box=detection['xyxy'], class_id=detection['class_id'], hits=hits))
            if hits >= self.frames:
                keep.append(index)
        self.previous, self.last_stamp, self.last_wall_time = current, stamp, now
        return sorted(keep)
