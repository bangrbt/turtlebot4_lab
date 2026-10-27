"""Pure perception policy, temporal Wave detector and strict wire protocol."""
from collections import deque
from dataclasses import dataclass
import json
import math

LABELS = {'None', 'Wave', 'Pointing_Up', 'Victory', 'Thumb_Up', 'Open_Palm', 'Closed_Fist'}
SELECTORS = {'Wave': 'greeting', 'Pointing_Up': 'back_and_forth', 'Victory': 'circle'}


@dataclass(frozen=True)
class Observation:
    label: str
    confidence: float
    raw_label: str
    raw_confidence: float
    hand_visible: bool
    stable: bool
    valid_control: bool
    session_id: str
    sequence: int
    captured_unix: float
    reason: str = ''

    def dumps(self):
        return json.dumps(self.__dict__, allow_nan=False)

    @classmethod
    def loads(cls, value, unix_now, max_age=.45):
        d = json.loads(value)
        if not isinstance(d, dict) or set(d) != set(cls.__dataclass_fields__):
            raise ValueError('Gesture message schema mismatch')
        if not isinstance(d['label'], str) or d['label'] not in LABELS or not isinstance(d['raw_label'], str):
            raise ValueError('Unknown gesture')
        for key in ('confidence', 'raw_confidence'):
            if type(d[key]) not in (float, int) or not math.isfinite(d[key]) or not 0 <= d[key] <= 1:
                raise ValueError('Invalid confidence')
        for key in ('hand_visible', 'stable', 'valid_control'):
            if type(d[key]) is not bool:
                raise ValueError('Boolean expected')
        if type(d['sequence']) is not int or d['sequence'] < 0:
            raise ValueError('Invalid sequence')
        if not isinstance(d['session_id'], str) or not 8 <= len(d['session_id']) <= 80:
            raise ValueError('Invalid camera session')
        t = d['captured_unix']
        if type(t) not in (float, int) or not math.isfinite(t) or not -.10 <= unix_now-t <= max_age:
            raise ValueError('Stale/future source image')
        if not isinstance(d['reason'], str) or len(d['reason']) > 200:
            raise ValueError('Invalid reason')
        return cls(**d)


def distance(a, b):
    return math.sqrt(sum((u-v)**2 for u, v in zip(a, b)))


def joint_angle(a, b, c):
    u, v = [a[i]-b[i] for i in range(3)], [c[i]-b[i] for i in range(3)]
    denom = math.sqrt(sum(x*x for x in u)*sum(x*x for x in v))
    return math.degrees(math.acos(max(-1., min(1., sum(x*y for x, y in zip(u, v))/max(denom, 1e-9)))))


def geometry_matches(label, points):
    """Cross-check the classifier, never create a replacement label from geometry."""
    if len(points) != 21 or not all(math.isfinite(v) for p in points for v in p):
        return False
    if distance(points[5], points[17]) < .035:
        return False  # Hand too small to command reliably.
    straight = []
    for mcp in (5, 9, 13, 17):
        straight.append(joint_angle(points[mcp], points[mcp+1], points[mcp+3]) > 140 and
                        distance(points[0], points[mcp+3]) > distance(points[0], points[mcp+1])*1.05)
    if label == 'Pointing_Up':
        return straight == [True, False, False, False] and points[8][1] < points[5][1]-.02
    if label == 'Victory':
        return straight == [True, True, False, False]
    if label == 'Open_Palm':
        return all(straight)
    if label == 'Closed_Fist':
        return not any(straight) and all(distance(points[0], points[i+3]) < 1.3*distance(points[0], points[i+1]) for i in (5,9,13,17))
    if label == 'Thumb_Up':
        thumb_extended = joint_angle(points[2], points[3], points[4]) > 115
        thumb_upward = points[4][1] < points[2][1] and (points[4][1] < points[3][1] or points[4][1] < points[5][1])
        other_folded = not any(straight[1:]) and (not straight[0] or points[8][1] > points[4][1])
        return thumb_extended and thumb_upward and other_folded
    return False


class StableGesture:
    def __init__(self, frames=4, hold=.30, max_gap=.25):
        if frames < 2 or hold <= 0 or max_gap <= 0:
            raise ValueError('Invalid stability settings')
        self.frames, self.hold, self.max_gap = frames, hold, max_gap
        self.reset()

    def reset(self):
        self.label, self.first, self.last, self.count = 'None', 0., None, 0

    def update(self, label, now):
        if label == 'None' or (self.last is not None and (now <= self.last or now-self.last > self.max_gap)):
            self.reset()
        if label != self.label:
            self.label, self.first, self.count = label, now, 0
        self.count += 1
        self.last = now
        return label != 'None' and self.count >= self.frames and now-self.first >= self.hold


class WaveDetector:
    """Open-palm lateral motion: one clear lateral reversal; reject a static palm and small jitter."""
    def __init__(self, window=1.4, latch=.35):
        self.window, self.latch = window, latch
        self.reset()

    def reset(self):
        self.samples = deque()
        self.last_wave, self.last_turn = -math.inf, -math.inf

    def update(self, now, x, y, width):
        if self.samples and (now <= self.samples[-1][0] or now-self.samples[-1][0] > .50):
            self.reset()
        self.samples.append((now, x, y, width))
        while self.samples and now-self.samples[0][0] > self.window:
            self.samples.popleft()
        samples = list(self.samples)
        if len(samples) >= 5 and samples[-1][0]-samples[0][0] >= .30:
            span = sorted(s[3] for s in samples)[len(samples)//2]
            xs, ys = [s[1] for s in samples], [s[2] for s in samples]
            threshold, direction, extreme, turns, turn_at = .28*span, 0, xs[0], 0, -math.inf
            for t, x_val, _, _ in samples[1:]:
                delta = x_val-extreme
                if direction == 0:
                    if abs(delta) >= threshold:
                        direction, extreme = (1 if delta > 0 else -1), x_val
                elif direction*delta >= 0:
                    extreme = x_val
                elif abs(delta) >= threshold:
                    direction, extreme, turns, turn_at = -direction, x_val, turns+1, t
            if (turns >= 1 and max(xs)-min(xs) >= .60*span and max(ys)-min(ys) <= 1.5*span
                    and turn_at > self.last_turn):
                self.last_wave, self.last_turn = now, turn_at
        return now-self.last_wave <= self.latch


class RecognitionPolicy:
    def __init__(self, confidence=.80, margin=.15, confirm_confidence=.90, frames=4, hold=.30):
        if not 0 < confidence <= confirm_confidence <= 1 or not 0 <= margin < 1:
            raise ValueError('Invalid recognition thresholds')
        self.confidence, self.margin, self.confirm_confidence = confidence, margin, confirm_confidence
        self.stability, self.wave = StableGesture(frames, hold), WaveDetector()
        self.last_center = None

    def reset(self):
        self.stability.reset(); self.wave.reset(); self.last_center = None

    def decide(self, hands, now):
        # hands: [(sorted (label,score) pairs, 21 xyz landmarks)]. No ROS/model dependency.
        for categories, _ in hands:
            if categories and categories[0][0] == 'Open_Palm' and categories[0][1] >= .55:
                if len(hands) != 1:
                    self.reset()
                    return dict(label='Open_Palm', confidence=categories[0][1], raw_label='Open_Palm',
                                raw_confidence=categories[0][1], hand_visible=True, stable=True, valid_control=True, reason='stop_priority')
        if len(hands) > 1:
            spans = [distance(pts[0], pts[9]) for _, pts in hands]
            max_s = max(spans)
            max_i = spans.index(max_s)
            others = [s for i, s in enumerate(spans) if i != max_i]
            if max_s >= 0.08 and all(max_s >= 1.35 * s for s in others):
                hands = [hands[max_i]]
        if len(hands) != 1:
            self.reset()
            return self.unknown('no_hand' if not hands else 'multiple_hands', bool(hands))
        categories, points = hands[0]
        if not categories:
            self.reset(); return self.unknown('no_class', True)
        label, score = categories[0]
        runner = categories[1][1] if len(categories)>1 else 0.
        raw = dict(raw_label=label, raw_confidence=score, hand_visible=True)
        if label == 'Closed_Fist' and score >= .65:
            self.reset(); return dict(label=label, confidence=score, stable=True, valid_control=True, reason='cancel_priority', **raw)
        palm_stop = label == 'Open_Palm' and score >= .55 and geometry_matches('Open_Palm', points)
        threshold = self.confirm_confidence if label == 'Thumb_Up' else self.confidence
        valid = label in LABELS-{'None','Wave'} and score >= threshold and score-runner >= self.margin and geometry_matches(label, points)
        if not valid:
            self.reset()
            if palm_stop:
                return dict(label='Open_Palm',confidence=score,stable=True,valid_control=True,reason='stop_priority',**raw)
            return dict(label='None',confidence=score,stable=False,valid_control=False,reason='uncertain_or_geometry',**raw)
        center = tuple(sum(points[i][j] for i in (0,5,9,13,17))/5 for j in (0,1))
        if self.last_center and math.dist(center,self.last_center) > .30:
            self.reset()
        self.last_center = center
        if label == 'Open_Palm':
            waving = self.wave.update(now, *center, distance(points[5],points[17]))
            if waving:
                # Temporal motion itself is the confirmation: do not add a static hold.
                self.stability.reset()
                return dict(label='Wave',confidence=score,stable=True,valid_control=True,
                            reason='wave_detected',**raw)
            else:
                self.stability.reset()
                return dict(label='Open_Palm',confidence=score,stable=True,valid_control=True,reason='stop_or_wave_sampling',**raw)
        else:
            self.wave.reset()
        stable = self.stability.update(label, now)
        return dict(label=label,confidence=score,stable=stable,valid_control=True,
                    reason='stable' if stable else 'confirming',**raw)

    @staticmethod
    def unknown(reason, hand_visible=False):
        return dict(label='None',confidence=0.,raw_label='None',raw_confidence=0.,hand_visible=hand_visible,
                    stable=False,valid_control=False,reason=reason)
