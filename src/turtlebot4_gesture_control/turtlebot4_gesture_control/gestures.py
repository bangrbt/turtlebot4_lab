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


def hand_shape(points):
    """Orientation-independent geometry for the two priority gestures.

    Finger-only closure also describes Thumb Up: the thumb must be tucked close
    to the index knuckle before the landmark fallback may cancel anything.
    """
    if len(points) != 21 or any(len(p) != 3 for p in points):
        return False, False, 0.
    if not all(math.isfinite(v) for p in points for v in p):
        return False, False, 0.
    palm = distance(points[0], points[9])
    width = distance(points[5], points[17])
    if palm < .04 or width < .025:
        return False, False, 0.
    extended, folded = [], []
    for mcp in (5, 9, 13, 17):
        angle = joint_angle(points[mcp], points[mcp+1], points[mcp+3])
        ratio = distance(points[0], points[mcp+3])/max(distance(points[0], points[mcp+1]), 1e-6)
        extended.append(angle > 125 and ratio > 1.03)
        folded.append(angle < 125 and ratio < 1.10)
    tucked_thumb = (distance(points[4], points[5]) < .60*palm and
                    distance(points[4], points[0]) < 1.35*palm)
    return sum(extended) >= 3, all(folded) and tucked_thumb, max(width, .45*palm)


class WaveDetector:
    """A short, clear out-and-back movement; retain motion across brief label flicker.

    The active wave lasts while the same palm is moving. A static palm clears
    the history, so an old turn cannot be replayed as a new greeting.
    """
    def __init__(self, window=1.2, still_seconds=.25, max_gap=.40):
        self.window, self.still_seconds, self.max_gap = window, still_seconds, max_gap
        self.reset()

    def reset(self):
        self.samples = deque()
        self.active = False
        self.last_motion = None
        self.stationary = False

    def update(self, now, x, y, width):
        if not all(math.isfinite(v) for v in (now, x, y, width)) or width < .025:
            self.reset()
            return False
        if self.samples and (now <= self.samples[-1][0] or now-self.samples[-1][0] > self.max_gap):
            self.reset()
        if not self.samples:
            self.last_motion = now
            self.motion_anchor = (x, y)
        elif math.dist((x, y), self.motion_anchor) >= .08*width:
            self.last_motion = now
            self.motion_anchor = (x, y)
        self.samples.append((now, x, y, width))
        while self.samples and now-self.samples[0][0] > self.window:
            self.samples.popleft()
        samples = list(self.samples)
        self.stationary = len(samples) >= 2 and now-self.last_motion >= self.still_seconds
        if self.stationary:
            # Do not keep the old reversal in a sliding window after the wave ends.
            self.active = False
            self.samples = deque([samples[-1]])
            return False
        if len(samples) >= 4 and samples[-1][0]-samples[0][0] >= .18:
            span = sorted(s[3] for s in samples)[len(samples)//2]
            xs, ys = [s[1] for s in samples], [s[2] for s in samples]
            threshold, direction, extreme, turns = .20*span, 0, xs[0], 0
            for _, x_val, _, _ in samples[1:]:
                delta = x_val-extreme
                if direction == 0:
                    if abs(delta) >= threshold:
                        direction, extreme = (1 if delta > 0 else -1), x_val
                elif direction*delta >= 0:
                    extreme = x_val
                elif abs(delta) >= threshold:
                    direction, extreme, turns = -direction, x_val, turns+1
            if turns >= 1 and max(xs)-min(xs) >= .45*span and max(ys)-min(ys) <= 1.5*span:
                self.active = True
        return self.active


class RecognitionPolicy:
    def __init__(self, confidence=.80, margin=.15, confirm_confidence=.90, frames=4, hold=.30,
                 fist_confidence=.50):
        if not 0 < confidence <= confirm_confidence <= 1 or not 0 <= margin < 1 or not 0 < fist_confidence <= 1:
            raise ValueError('Invalid recognition thresholds')
        self.confidence, self.margin, self.confirm_confidence = confidence, margin, confirm_confidence
        self.fist_confidence = fist_confidence
        self.stability, self.wave = StableGesture(frames, hold), WaveDetector()
        self.fist_gate = StableGesture(frames=2, hold=.06, max_gap=.35)
        self.last_center = None

    def reset(self):
        self.stability.reset()
        self.wave.reset()
        self.fist_gate.reset()
        self.last_center = None

    def cancel(self, categories, reason):
        self.stability.reset()
        self.wave.reset()
        self.last_center = None
        raw_label, raw_score = categories[0] if categories else ('None', 0.)
        # A landmark fallback has no classifier probability for Closed_Fist.
        # Preserve the real raw prediction and never invent an AI confidence.
        return dict(label='Closed_Fist', confidence=raw_score if raw_label == 'Closed_Fist' else 0.,
                    raw_label=raw_label, raw_confidence=raw_score, hand_visible=True,
                    stable=True, valid_control=True, reason=reason)

    def decide(self, hands, now):
        # CANCEL is checked for EVERY visible hand, before palm/ambiguity/Wave.
        for categories, _ in hands:
            if categories and categories[0][0] == 'Closed_Fist' and categories[0][1] >= self.fist_confidence:
                self.fist_gate.reset()
                return self.cancel(categories, 'cancel_priority')
        fist = next(((classes, pts) for classes, pts in hands if hand_shape(pts)[1]), None)
        if fist is not None:
            if self.fist_gate.update('Closed_Fist', now):
                return self.cancel(fist[0], 'fist_landmarks_cancel')
        else:
            self.fist_gate.reset()

        if len(hands) > 1:
            for categories, _ in hands:
                if categories and categories[0][0] == 'Open_Palm' and categories[0][1] >= .55:
                    self.reset()
                    return dict(label='Open_Palm', confidence=categories[0][1], raw_label='Open_Palm',
                                raw_confidence=categories[0][1], hand_visible=True, stable=True,
                                valid_control=True, reason='stop_priority')
            spans = [distance(pts[0], pts[9]) for _, pts in hands]
            largest = max(range(len(spans)), key=spans.__getitem__)
            if spans[largest] >= .08 and all(spans[largest] >= 1.35*s for i, s in enumerate(spans) if i != largest):
                hands = [hands[largest]]
        if len(hands) != 1:
            self.stability.reset()
            if len(hands) > 1:
                self.wave.reset()
                self.last_center = None
            # A brief no-hand frame does not destroy an in-progress wave. A gap
            # longer than max_gap resets automatically at the next real sample.
            return self.unknown('no_hand' if not hands else 'multiple_hands', bool(hands))
        categories, points = hands[0]
        label, score = categories[0] if categories else ('None', 0.)
        raw = dict(raw_label=label, raw_confidence=score, hand_visible=True)
        opened, _, width = hand_shape(points)
        if opened:
            # Finger tips respond to wrist waves even when the palm center barely moves.
            center = tuple(.2*sum(points[i][j] for i in (0,5,9,13,17))/5 +
                           .8*sum(points[i][j] for i in (8,12,16,20))/4 for j in (0,1))
            if self.last_center and math.dist(center, self.last_center) > max(.25, 2.*width):
                self.wave.reset()
            self.last_center = center
            waving = self.wave.update(now, *center, width)
            self.stability.reset()
            if waving:
                return dict(label='Wave', confidence=score if label == 'Open_Palm' else 0.,
                            stable=True, valid_control=True, reason='wave_landmarks', **raw)
            if label == 'Open_Palm' and score >= .55:
                return dict(label='Open_Palm', confidence=score, stable=True, valid_control=True,
                            reason='palm_stationary' if self.wave.stationary else 'palm_motion', **raw)
            return self.unknown_with_raw('palm_motion', raw, score)

        # Keep an active moving wave across one imperfect pose/label, but never
        # mistake that imperfect frame for another Wave command or a stationary STOP.
        if self.wave.samples and now-self.wave.samples[-1][0] > self.wave.max_gap:
            self.wave.reset()
            self.last_center = None
        if label in {'Thumb_Up', 'Pointing_Up', 'Victory', 'Closed_Fist'}:
            self.wave.reset()
            self.last_center = None
        if label == 'Open_Palm' and score >= .55:
            return self.unknown_with_raw('palm_pose_uncertain', raw, score)
        threshold = self.confirm_confidence if label == 'Thumb_Up' else self.confidence
        runner = categories[1][1] if len(categories) > 1 else 0.
        valid = (label in LABELS-{'None','Wave','Closed_Fist'} and score >= threshold and
                 score-runner >= self.margin and geometry_matches(label, points))
        if not valid:
            self.stability.reset()
            return self.unknown_with_raw('uncertain_or_geometry', raw, score)
        stable = self.stability.update(label, now)
        return dict(label=label, confidence=score, stable=stable, valid_control=True,
                    reason='stable' if stable else 'confirming', **raw)

    @staticmethod
    def unknown_with_raw(reason, raw, score):
        return dict(label='None', confidence=score, stable=False, valid_control=False, reason=reason, **raw)

    @staticmethod
    def unknown(reason, hand_visible=False):
        return dict(label='None', confidence=0., raw_label='None', raw_confidence=0., hand_visible=hand_visible,
                    stable=False, valid_control=False, reason=reason)
