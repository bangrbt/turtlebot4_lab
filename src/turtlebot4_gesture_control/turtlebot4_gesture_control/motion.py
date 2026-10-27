"""Odometry-based finite trajectories; no sleeps, ROS or wall-time dead reckoning."""
from dataclasses import dataclass
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    stamp: float


@dataclass(frozen=True)
class MotionConfig:
    linear_speed: float = .30
    angular_speed: float = 1.10
    leg_distance: float = .12
    cycles: int = 4
    circle_radius: float = .85
    greeting_angle: float = .55
    settle_seconds: float = .10

    def __post_init__(self):
        values = [self.linear_speed, self.angular_speed, self.leg_distance,
                  self.circle_radius, self.greeting_angle, self.settle_seconds]
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Motion values must be finite and positive')
        if not .04 <= self.linear_speed <= .35 or not .1 <= self.angular_speed <= 1.2:
            raise ValueError('Unsafe configured speed')
        if not .05 <= self.leg_distance <= 1.5 or not .2 <= self.circle_radius <= 2.0:
            raise ValueError('Trajectory does not fit the verified demo area')
        if type(self.cycles) is not int or not 1 <= self.cycles <= 8 or not .2 <= self.greeting_angle <= .8:
            raise ValueError('Invalid cycle count/greeting angle')


@dataclass(frozen=True)
class Command:
    linear: float = 0.
    angular: float = 0.
    done: bool = False
    failure: str = ''


class MotionExecutor:
    def __init__(self, kind, start, config=None):
        if kind not in {'greeting','back_and_forth','circle'}:
            raise ValueError('Unknown behavior')
        self.kind, self.start, self.config = kind, start, config or MotionConfig()
        self.phase, self.completed_cycles = 0, 0
        self.wait_until = -math.inf
        self.previous_yaw, self.swept = start.yaw, 0.
        self.progress_pose, self.progress_stamp = start, start.stamp
        self.started_stamp = start.stamp
        self.finished = False

    def sample(self, pose):
        c = self.config
        if self.finished:
            return Command(done=True)
        if not all(math.isfinite(v) for v in (pose.x,pose.y,pose.yaw,pose.stamp)):
            return Command(failure='invalid_odometry')
        displacement = math.hypot(pose.x-self.start.x, pose.y-self.start.y)
        limit = 2*c.circle_radius+.35 if self.kind == 'circle' else (c.leg_distance+.25 if self.kind == 'back_and_forth' else .15)
        if displacement > limit:
            return Command(failure='trajectory_deviation')
        if pose.stamp-self.started_stamp > 150:
            return Command(failure='behavior_simulation_timeout')
        turn = wrap(pose.yaw-self.previous_yaw)
        self.swept += turn
        self.previous_yaw = pose.yaw
        if pose.stamp < self.wait_until:
            return Command()
        if self.kind == 'greeting':
            offsets = [c.greeting_angle,-c.greeting_angle,0.]
            error = wrap(self.start.yaw+offsets[self.phase]-pose.yaw)
            if abs(error) <= .025:
                return self.advance(pose, len(offsets))
            # Strong turn, short deceleration: a greeting should be immediately visible.
            speed = min(c.angular_speed, max(.35, 4.0*abs(error)))
            command = Command(angular=math.copysign(speed,error))
        elif self.kind == 'back_and_forth':
            forward = self.phase % 2 == 0
            d = c.leg_distance if forward else 0.
            tx,ty = self.start.x+d*math.cos(self.start.yaw),self.start.y+d*math.sin(self.start.yaw)
            dx,dy = tx-pose.x,ty-pose.y
            along = dx*math.cos(self.start.yaw)+dy*math.sin(self.start.yaw)
            cross = -math.sin(self.start.yaw)*(pose.x-self.start.x)+math.cos(self.start.yaw)*(pose.y-self.start.y)
            if abs(cross) > max(.12, .35*c.leg_distance):
                return Command(failure='lateral_deviation')
            error = wrap(self.start.yaw-pose.yaw)
            if math.hypot(dx,dy) <= max(.020, .07*c.leg_distance) and abs(error) <= .08:
                return self.advance(pose, 2*c.cycles)
            velocity = max(-c.linear_speed,min(c.linear_speed,2.5*along)) if abs(error)<.20 else 0.
            command = Command(velocity,max(-c.angular_speed,min(c.angular_speed,1.8*error-1.2*cross*(1 if forward else -1))))
        else:
            remaining = 2*math.pi-self.swept
            if remaining <= .025 and displacement <= max(.10, .15*c.circle_radius):
                self.finished = True
                return Command(done=True)
            if self.swept > 2*math.pi+.15:
                return Command(failure='circle_closure_error')
            cx = self.start.x-c.circle_radius*math.sin(self.start.yaw)
            cy = self.start.y+c.circle_radius*math.cos(self.start.yaw)
            radius_error = math.hypot(pose.x-cx,pose.y-cy)-c.circle_radius
            if abs(radius_error) > max(.12, .22*c.circle_radius):
                return Command(failure='circle_radius_error')
            tangent = math.atan2(pose.y-cy,pose.x-cx)+math.pi/2
            v = min(c.linear_speed,max(.035,remaining*c.circle_radius*.8))
            omega = v/c.circle_radius+1.5*wrap(tangent-pose.yaw)+.9*radius_error
            command = Command(v,max(-c.angular_speed,min(c.angular_speed,omega)))
        moved = math.hypot(pose.x-self.progress_pose.x,pose.y-self.progress_pose.y)
        rotated = abs(wrap(pose.yaw-self.progress_pose.yaw))
        if moved >= .012 or rotated >= .025:
            self.progress_pose,self.progress_stamp=pose,pose.stamp
        elif pose.stamp-self.progress_stamp > 4.:
            return Command(failure='motion_stalled')
        return command

    def advance(self, pose, total):
        self.phase += 1
        if self.kind == 'back_and_forth':
            self.completed_cycles = self.phase//2
        self.progress_pose,self.progress_stamp=pose,pose.stamp
        if self.phase >= total:
            self.finished = True
            return Command(done=True)
        self.wait_until = pose.stamp+self.config.settle_seconds
        return Command()
