"""One-shot gesture commands with a camera-source watchdog, independent of hand pose."""
from dataclasses import dataclass
import math
from .gestures import SELECTORS, StableGesture
from .motion import Command, MotionConfig, MotionExecutor


@dataclass(frozen=True)
class BehaviorConfig:
    signal_timeout: float = .80
    selection_timeout: float = 12.
    max_behavior_wall_seconds: float = 180.
    confidence: float = .55
    confirmation_confidence: float = .60
    fist_confidence: float = .50

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in self.__dict__.values()):
            raise ValueError('Invalid behavior parameter')
        if (not .2 <= self.signal_timeout <= 1 or not self.confidence <= self.confirmation_confidence <= 1
                or not 0 < self.fist_confidence <= 1):
            raise ValueError('Invalid watchdog/confidence')


class BehaviorMachine:
    SOURCE_FAILURES = {'camera_error', 'camera_timeout', 'inference_stale',
                       'recognizer_shutdown', 'preview_only'}

    def __init__(self, config=None, motion=None):
        self.config, self.motion = config or BehaviorConfig(), motion or MotionConfig()
        self.state, self.selected, self.reason = 'IDLE', None, 'waiting_for_selection'
        self.session, self.sequence = None, -1
        self.last_frame, self.selected_at, self.started_at = -math.inf, 0., 0.
        self.gate = StableGesture(frames=3, hold=.15)
        self.confirm_gate = StableGesture(frames=3, hold=.25)
        self.last_event = None
        self.wave_latched = False
        self.hand_absent_since = None
        self.executor = None
        self.completions = 0
        self.last_completed_cycles = 0

    def stop(self, reason):
        self.state, self.selected, self.executor, self.reason = 'IDLE', None, None, reason
        self.gate.reset()
        self.confirm_gate.reset()
        self.last_event = None
        return Command()

    def start(self, kind, now, pose, start_reason):
        if start_reason or pose is None:
            self.stop(start_reason or 'odometry_missing')
            return
        self.selected = kind
        self.executor = MotionExecutor(kind, pose, self.motion)
        self.state, self.reason, self.started_at = 'RUNNING', 'executing', now
        self.last_completed_cycles = 0

    def observe(self, obs, now, pose, start_reason=None):
        if obs.session_id != self.session:
            self.stop('camera_session_changed')
            self.wave_latched = False
            self.session, self.sequence = obs.session_id, -1
        if obs.sequence <= self.sequence:
            return False
        self.sequence = obs.sequence
        if obs.reason in self.SOURCE_FAILURES:
            self.stop('camera_source_failed:' + obs.reason)
            return True

        # A NEW, timestamp-validated image proves that the camera pipeline is alive.
        # No hand / uncertain class is not a camera failure and does not cancel a command.
        self.last_frame = now
        if not obs.hand_visible:
            if self.hand_absent_since is None:
                self.hand_absent_since = now
            if now - self.hand_absent_since >= .30:
                self.wave_latched = False
        else:
            self.hand_absent_since = None

        # Accepted CANCEL always wins, including landmark fallback / low raw score.
        if ((obs.label == 'Closed_Fist' and obs.valid_control) or
                (obs.raw_label == 'Closed_Fist' and obs.raw_confidence >= self.config.fist_confidence)):
            self.wave_latched = False
            self.stop('closed_fist_cancel')
            return True

        wave = (obs.label == 'Wave' and obs.stable and obs.valid_control and
                obs.hand_visible and (obs.confidence >= self.config.confidence or obs.reason == 'wave_landmarks'))
        if wave:
            self.gate.reset()
            self.confirm_gate.reset()
            if not self.wave_latched:
                self.wave_latched = True
                self.last_event = 'Wave'
                # Temporal recognition already proves a wave; no second static hold/Thumb Up.
                if not (self.state == 'RUNNING' and self.selected == 'greeting'):
                    self.start('greeting', now, pose, start_reason)
                return True
            return False

        # A wave can contain a low-score / imperfect Open Palm frame. Do not
        # interrupt its greeting until a genuinely stationary palm is observed.
        palm_pending = obs.reason in {'palm_motion', 'palm_pose_uncertain'}
        if obs.raw_label == 'Open_Palm' and obs.raw_confidence >= .55:
            if self.state == 'RUNNING' and self.selected == 'greeting' and palm_pending:
                return False
            self.wave_latched = False
            self.stop('open_palm_stop')
            return True

        threshold = self.config.confirmation_confidence if obs.label == 'Thumb_Up' else self.config.confidence
        if (not obs.hand_visible or not obs.valid_control or obs.confidence < threshold
                or obs.label == 'None' or not obs.stable):
            self.gate.reset()
            self.confirm_gate.reset()
            return False
        self.wave_latched = False
        if obs.label == 'Thumb_Up':
            self.gate.reset()
            event = self.confirm_gate.update(obs.label, now)
        else:
            self.confirm_gate.reset()
            event = self.gate.update(obs.label, now)
        if not event or self.last_event == obs.label:
            return False
        self.last_event = obs.label
        if obs.label in SELECTORS:
            kind = SELECTORS[obs.label]
            if self.state == 'RUNNING' and self.selected == kind:
                return False
            # Another selector interrupts the old behavior and waits for a new confirmation.
            self.executor = None
            self.selected, self.selected_at = kind, now
            self.state, self.reason = 'SELECTED', 'show_thumbs_up_to_confirm'
        elif obs.label == 'Thumb_Up' and self.state != 'RUNNING':
            if self.state != 'SELECTED' or self.selected is None:
                self.reason = 'confirmation_without_selection'
            elif now - self.selected_at > self.config.selection_timeout:
                self.stop('selection_expired')
            else:
                self.start(self.selected, now, pose, start_reason)
        return True

    def tick(self, now, pose, safety_reason=None):
        if self.state == 'RUNNING' and now - self.last_frame > self.config.signal_timeout:
            return self.stop('camera_signal_lost')
        if self.state == 'SELECTED' and now - self.selected_at > self.config.selection_timeout:
            return self.stop('selection_expired')
        if self.state != 'RUNNING':
            return Command()
        if safety_reason:
            return self.stop(safety_reason)
        if now - self.started_at > self.config.max_behavior_wall_seconds:
            return self.stop('behavior_wall_timeout')
        if pose is None:
            return self.stop('odometry_missing')
        command = self.executor.sample(pose)
        if command.failure:
            return self.stop(command.failure)
        if command.done:
            self.completions += 1
            self.last_completed_cycles = self.executor.completed_cycles
            # Both a held confirmation and a held Wave must not restart a completed behavior.
            self.state, self.selected, self.executor, self.reason = 'IDLE', None, None, 'behavior_completed'
            return Command()
        return command
