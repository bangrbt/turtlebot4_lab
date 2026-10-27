"""ROS adapter: independent steady-time watchdog, odometry and surround guard."""
import json
import math
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import String, Bool
from irobot_create_msgs.msg import HazardDetectionVector
from .gestures import Observation
from .behavior import BehaviorConfig, BehaviorMachine
from .motion import Pose, MotionConfig
from .safety import scan_clearance


class RobotBehaviorNode(Node):
    def __init__(self):
        super().__init__('robot_behavior_node')
        defaults=dict(enable_motion=True, signal_timeout=.80, selection_timeout=12., max_behavior_wall_seconds=180.,
                      confidence=.55, confirmation_confidence=.60, linear_speed=.30, angular_speed=1.10,
                      leg_distance=.12, cycles=4, circle_radius=.85, greeting_angle=.55, settle_seconds=.10,
                      scan_timeout=1., odometry_timeout=.75, clock_timeout=.75, hazard_timeout=1.5,
                      min_clearance=.45, start_clearance=2.2, require_arena_ready=True,
                      gesture_topic='/detected_gesture',cmd_vel_topic='/cmd_vel')
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}
        for key in ('scan_timeout','odometry_timeout','clock_timeout','hazard_timeout','min_clearance','start_clearance'):
            if not math.isfinite(self.p[key]) or self.p[key]<=0:raise ValueError('Invalid '+key)
        motion=MotionConfig(**{k:self.p[k] for k in MotionConfig.__dataclass_fields__})
        if self.p['min_clearance'] < .35 or self.p['start_clearance'] < max(2*motion.circle_radius+.40,motion.leg_distance+.4):
            raise ValueError('Configured clearance cannot cover the motion envelope')
        self.machine=BehaviorMachine(BehaviorConfig(**{k:self.p[k] for k in BehaviorConfig.__dataclass_fields__}),motion)
        self.pose,self.clearance=None,None
        self.odom_seen,self.scan_seen,self.clock_seen,self.hazard_seen=-math.inf,-math.inf,-math.inf,-math.inf
        self.odom_stamp,self.scan_stamp,self.clock_stamp=-1.,-1.,-1.
        self.arena_seen=-math.inf;self.arena_ready=False;self.hazards=set();self.conflict=None;self.graph_at=0.
        self.last_status,self.status_at=None,0.
        self.velocity=self.create_publisher(Twist,self.p['cmd_vel_topic'],10)
        tl=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status=self.create_publisher(String,'/gesture/behavior_status',tl)
        self.inputs=[
            self.create_subscription(String,self.p['gesture_topic'],self.on_gesture,10),
            self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data),
            self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data),
            self.create_subscription(ClockMsg,'/clock',self.on_clock,qos_profile_sensor_data),
            self.create_subscription(HazardDetectionVector,'/hazard_detection',self.on_hazards,qos_profile_sensor_data),
            self.create_subscription(Bool,'/gesture/arena_ready',self.on_arena,tl)]
        self.timer=self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_velocity(0.,0.)

    @staticmethod
    def stamp(msg):return msg.sec+msg.nanosec/1e9

    def publish_velocity(self,linear,angular):
        msg=Twist();msg.linear.x=float(linear);msg.angular.z=float(angular);self.velocity.publish(msg)

    def stop(self,reason):
        self.machine.stop(reason);self.publish_velocity(0.,0.)

    def on_arena(self,msg):
        self.arena_ready=msg.data;self.arena_seen=time.monotonic()
        if not msg.data:self.stop('arena_not_ready')

    def on_clock(self,msg):
        t=self.stamp(msg.clock)
        if t<self.clock_stamp:
            self.stop('simulation_time_reset');self.pose=None;self.odom_stamp=self.scan_stamp=-1.
            self.odom_seen=self.scan_seen=-math.inf
        if t>self.clock_stamp:self.clock_seen=time.monotonic()
        self.clock_stamp=t

    def on_odom(self,msg):
        t=self.stamp(msg.header.stamp)
        if t<=self.odom_stamp:return
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        vals=(p.x,p.y,q.x,q.y,q.z,q.w)
        if not all(math.isfinite(v) for v in vals) or abs(sum(v*v for v in vals[2:])-1.)>.05:
            self.stop('invalid_odometry');return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.pose=Pose(p.x,p.y,yaw,t);self.odom_stamp=t;self.odom_seen=time.monotonic()

    def on_scan(self,msg):
        t=self.stamp(msg.header.stamp)
        if t<=self.scan_stamp:return
        self.clearance=scan_clearance(msg.ranges,msg.range_min,msg.range_max,msg.angle_increment)
        self.scan_stamp,self.scan_seen=t,time.monotonic()
        if self.clearance is None or self.clearance<self.p['min_clearance']:
            self.stop('scan_invalid' if self.clearance is None else 'obstacle_too_close')

    def on_hazards(self,msg):
        self.hazards={int(h.type) for h in msg.detections};self.hazard_seen=time.monotonic()
        if self.hazards-{0}:self.stop('hazard_detected')

    def safety_reason(self,starting=False):
        now=time.monotonic()
        if not self.p['enable_motion']:return 'motion_disabled'
        if self.p['require_arena_ready'] and (not self.arena_ready or now-self.arena_seen>1.5):return 'arena_not_ready'
        if now-self.clock_seen>self.p['clock_timeout']:return 'clock_missing_or_paused'
        if self.pose is None or now-self.odom_seen>self.p['odometry_timeout']:return 'odometry_stale'
        if now-self.scan_seen>self.p['scan_timeout'] or self.clearance is None:return 'scan_missing_or_invalid'
        if self.clearance<(self.p['start_clearance'] if starting else self.p['min_clearance']):return 'insufficient_clearance'
        if now-self.hazard_seen>self.p['hazard_timeout']:return 'hazard_stream_missing'
        if self.hazards-{0}:return 'hazard_detected'
        return self.conflict

    def on_gesture(self,msg):
        try:obs=Observation.loads(msg.data,time.time())
        except (ValueError,TypeError,KeyError,json.JSONDecodeError):
            self.stop('invalid_or_stale_gesture');return
        old_executor = self.machine.executor
        self.machine.observe(obs,time.monotonic(),self.pose,self.safety_reason(starting=True))
        if self.machine.state != 'RUNNING' or old_executor is not self.machine.executor:
            self.publish_velocity(0.,0.)

    def tick(self):
        now=time.monotonic()
        if now-self.graph_at>1.:
            self.graph_at=now;names=self.get_node_names()
            bad=[n for n in names if n in {'controller_server','velocity_smoother','lab05_sequential_navigation'} or 'teleop' in n]
            self.conflict='conflicting_controller:'+','.join(sorted(set(bad))) if bad else None
            if names.count(self.get_name())>1:self.conflict='duplicate_behavior_controller'
        command=self.machine.tick(now,self.pose,self.safety_reason())
        self.publish_velocity(command.linear,command.angular)
        executor=self.machine.executor
        status=dict(state=self.machine.state,selected=self.machine.selected,reason=self.machine.reason,
                    ready=self.safety_reason(starting=True) is None,safety_reason=self.safety_reason(starting=True),
                    phase=executor.phase if executor else None,completed_cycles=executor.completed_cycles if executor else self.machine.last_completed_cycles,
                    completions=self.machine.completions,linear=command.linear,angular=command.angular,
                    clearance=self.clearance,hazards=sorted(self.hazards),camera_age=now-self.machine.last_frame if math.isfinite(self.machine.last_frame) else None)
        key=(status['state'],status['selected'],status['reason'],status['safety_reason'],status['completions'])
        if key!=self.last_status or now-self.status_at>.25:
            self.status.publish(String(data=json.dumps(status)))
            self.status_at=now
        if key!=self.last_status:self.get_logger().info(json.dumps(status));self.last_status=key

    def close(self):
        self.stop('shutdown')
        for _ in range(4):
            self.publish_velocity(0.,0.)
            rclpy.spin_once(self,timeout_sec=.02)


def main(args=None):
    rclpy.init(args=args);node=None;stopped=False
    def stop(signum,frame):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        node=RobotBehaviorNode()
        while rclpy.ok() and not stopped:rclpy.spin_once(node,timeout_sec=.05)
        return 0
    except Exception as exc:
        print(f'Behavior error: {exc}',flush=True);return 1
    finally:
        if node:node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
