#!/usr/bin/env python3
"""Observe RViz goals and query their action result; never sends a goal or velocity."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path,help='Append-only JSONL evidence file')
    args=p.parse_args()
    path=args.output.expanduser()
    path.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    node=rclpy.create_node('turtlebot4_lab_goal_recorder')
    client=node.create_client(NavigateToPose.Impl.GetResultService,'/navigate_to_pose/_action/get_result')
    states={}
    pending=[]
    observed_goals=[]
    planned_endpoints=[]
    def log(record):
        record['wall_time_unix']=time.time()
        record['sim_time_ns']=node.get_clock().now().nanoseconds
        text=json.dumps(record,ensure_ascii=False)
        with path.open('a') as stream:
            stream.write(text+'\n')
        print(text,flush=True)
    def goal(msg):
        observed_goals.append(message_to_ordereddict(msg))
        log({'event':'rviz_goal_pose_observed','pose':observed_goals[-1]})
    def plan(msg):
        if not msg.poses:
            return
        endpoint=message_to_ordereddict(msg.poses[-1])
        # Nav2 Humble GoalTool sends through its panel, so /goal_pose can be absent.
        # A planner endpoint is supporting evidence, not the original action request.
        key=endpoint['pose']
        if not planned_endpoints or planned_endpoints[-1]['pose']!=key:
            planned_endpoints.append(endpoint)
            log({'event':'plan_endpoint_observed','endpoint':endpoint,
                 'source':'/plan last pose; compare navigator log and RViz screenshot'})
    def status(msg):
        for item in msg.status_list:
            uuid=bytes(item.goal_info.goal_id.uuid).hex()
            if states.get(uuid)==item.status:
                continue
            initial=uuid not in states
            states[uuid]=item.status
            log({'event':'action_status','goal_uuid':uuid,'status':item.status,
                 'goal_stamp':message_to_ordereddict(item.goal_info.stamp),
                 'latest_observed_goal_pose':observed_goals[-1] if observed_goals else None,
                 'latest_plan_endpoint':planned_endpoints[-1] if planned_endpoints else None,
                 'pose_association':'temporal hint only; match screenshot and UUID, especially for overlapping goals'})
            if item.status in [4,5,6] and client.service_is_ready():
                req=NavigateToPose.Impl.GetResultService.Request()
                req.goal_id=item.goal_info.goal_id
                pending.append((uuid,client.call_async(req)))
    subscriptions=[node.create_subscription(PoseStamped,'/goal_pose',goal,10),
        node.create_subscription(NavPath,'/plan',plan,10),
        node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',status,
                                 QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    log({'event':'recorder_started','note':'SUCCEEDED=4, CANCELED=5, ABORTED=6. Obstacle avoidance must be observed separately.'})
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=0.2)
            for entry in list(pending):
                uuid,future=entry
                if future.done():
                    try:
                        result=future.result()
                        log({'event':'action_result','goal_uuid':uuid,'response':message_to_ordereddict(result)})
                    except Exception as exc:
                        log({'event':'result_query_failed','goal_uuid':uuid,'error':str(exc)})
                    pending.remove(entry)
    except KeyboardInterrupt:
        log({'event':'recorder_stopped'})
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
