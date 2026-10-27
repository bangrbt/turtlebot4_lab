"""Deterministic state/kinematic checks. Does not touch webcam or a ROS graph."""
from dataclasses import replace
import math
import json
import pytest
from turtlebot4_gesture_control.gestures import (Observation,StableGesture,WaveDetector,RecognitionPolicy,geometry_matches)
from turtlebot4_gesture_control.motion import MotionExecutor,MotionConfig,Pose
from turtlebot4_gesture_control.behavior import BehaviorMachine
from turtlebot4_gesture_control.safety import scan_clearance


def observation(label='Pointing_Up',seq=1,**kw):
    d=dict(label=label,confidence=.98,raw_label=label,raw_confidence=.98,hand_visible=True,
           stable=True,valid_control=True,session_id='unit-test-session',sequence=seq,captured_unix=100.,reason='test')
    d.update(kw);return Observation(**d)


def feed(machine,label,t=0.,duration=.7,seq=0,**kw):
    end=t+duration
    while t<end:
        seq+=1;machine.observe(observation(label,seq,**kw),t,Pose(0,0,0,t));t+=.05
    return t,seq


def running(label='Pointing_Up'):
    m=BehaviorMachine();t,seq=feed(m,label)
    t,seq=feed(m,'Thumb_Up',t,seq=seq)
    assert m.state=='RUNNING';return m,t,seq


def test_selection_never_moves_without_confirmation():
    m=BehaviorMachine();t,seq=feed(m,'Pointing_Up')
    assert m.state=='SELECTED'
    for i in range(10):
        t+=.05;seq+=1;m.observe(observation(seq=seq),t,Pose(0,0,0,t))
        c=m.tick(t,Pose(0,0,0,t));assert c.linear==c.angular==0


def test_thumb_up_alone_never_arms():
    m=BehaviorMachine();feed(m,'Thumb_Up',duration=2.)
    assert m.state=='IDLE' and m.executor is None


@pytest.mark.parametrize('label,kind',[('Pointing_Up','back_and_forth'),('Victory','circle')])
def test_each_selector_requires_separate_confirmation(label,kind):
    m,t,seq=running(label);assert m.selected==kind


def test_low_confidence_and_unstable_confirmations_do_not_execute():
    m=BehaviorMachine();t,seq=feed(m,'Victory')
    t,seq=feed(m,'Thumb_Up',t,seq=seq,confidence=.59)
    assert m.state=='SELECTED'
    feed(m,'Thumb_Up',t,seq=seq,stable=False)
    assert m.state=='SELECTED'


@pytest.mark.parametrize('label,reason',[('Open_Palm','open_palm_stop'),('Closed_Fist','closed_fist_cancel')])
def test_stop_and_cancel_are_immediate_without_stability(label,reason):
    m,t,seq=running();m.observe(observation(label,seq+1,stable=False),t,Pose(0,0,0,t))
    assert m.state=='IDLE' and m.reason==reason and m.selected is None
    assert m.tick(t,Pose(0,0,0,t)).linear==0


def test_wave_preempts_current_behavior_without_confirmation():
    m,t,seq=running();m.observe(observation('Wave',seq+1,raw_label='Open_Palm'),t,Pose(0,0,0,t))
    assert m.state=='RUNNING' and m.executor.kind=='greeting'


def test_lowering_hand_keeps_confirmed_behavior_running():
    m,t,seq=running('Victory')
    for _ in range(25):
        t+=.05;seq+=1
        m.observe(observation('None',seq,hand_visible=False,valid_control=False,stable=False,reason='no_hand'),t,Pose(0,0,0,t))
        command=m.tick(t,Pose(.01,0,.01,t))
        assert m.state=='RUNNING' and command.linear>0


def test_lost_camera_messages_stop_and_do_not_auto_resume():
    m,t,seq=running('Victory')
    m.tick(t+1.,Pose(0,0,0,t+1.))
    assert m.state=='IDLE' and m.reason=='camera_signal_lost'
    feed(m,'Thumb_Up',t+1.,seq=seq)
    assert m.state=='IDLE'


@pytest.mark.parametrize('reason',['camera_error','camera_timeout','inference_stale','recognizer_shutdown','preview_only'])
def test_camera_fault_heartbeat_is_not_mistaken_for_no_hand(reason):
    m,t,seq=running()
    m.observe(observation('None',seq+1,hand_visible=False,valid_control=False,reason=reason),t,Pose(0,0,0,t))
    assert m.state=='IDLE' and m.reason=='camera_source_failed:'+reason


def test_uncertain_hand_does_not_cancel_an_already_confirmed_behavior():
    m,t,seq=running('Victory')
    for _ in range(25):
        t+=.05;seq+=1
        m.observe(observation('None',seq,confidence=.3,valid_control=False,stable=False,reason='uncertain_or_geometry'),t,Pose(0,0,0,t))
        m.tick(t,Pose(.01,0,.01,t))
        assert m.state=='RUNNING'


def test_duplicate_sequence_cannot_extend_heartbeat():
    m,t,seq=running();last=m.last_frame
    for i in range(25):
        m.observe(observation('Thumb_Up',seq),t+i*.05,Pose(0,0,0,t))
        m.tick(t+i*.05,Pose(0,0,0,t))
    assert m.last_frame==last and m.state=='IDLE'


def test_new_camera_session_disarms_running_behavior():
    m,t,seq=running();m.observe(observation('Thumb_Up',1,session_id='new-camera-session'),t,Pose(0,0,0,t))
    assert m.state=='IDLE' and m.executor is None


@pytest.mark.parametrize('fault',['clock_missing_or_paused','scan_missing_or_invalid','obstacle_too_close','hazard_detected','odometry_stale'])
def test_safety_fault_stops_and_requires_new_selection(fault):
    m,t,seq=running();c=m.tick(t,Pose(0,0,0,t),fault)
    assert m.state=='IDLE' and c.linear==c.angular==0 and m.reason==fault
    feed(m,'Thumb_Up',t,seq=seq);assert m.state=='IDLE'


def test_expired_selection_cannot_be_confirmed():
    m=BehaviorMachine();t,seq=feed(m,'Victory')
    m.tick(t+13,Pose(0,0,0,t+13));feed(m,'Thumb_Up',t+13,seq=seq)
    assert m.state=='IDLE'


def test_other_selector_interrupts_and_requires_new_confirmation():
    m,t,seq=running('Victory');t,seq=feed(m,'Pointing_Up',t,seq=seq)
    assert m.state=='SELECTED' and m.selected=='back_and_forth' and m.executor is None
    assert m.tick(t,Pose(0,0,0,t)).linear==0
    feed(m,'Thumb_Up',t,seq=seq)
    assert m.state=='RUNNING' and m.executor.kind=='back_and_forth'


def test_holding_confirmation_after_completion_cannot_restart():
    m,t,seq=running('Victory');m.executor.finished=True;m.tick(t,Pose(0,0,0,t))
    assert m.completions==1
    feed(m,'Thumb_Up',t,seq=seq,duration=3.)
    assert m.state=='IDLE' and m.completions==1


def test_unsafe_start_is_rejected_even_with_confirmed_gesture():
    m=BehaviorMachine();t,seq=feed(m,'Victory')
    for _ in range(15):
        seq+=1;t+=.05;m.observe(observation('Thumb_Up',seq),t,Pose(0,0,0,t),'insufficient_clearance')
    assert m.state=='IDLE' and m.executor is None


@pytest.mark.parametrize('kind',['greeting','back_and_forth','circle'])
@pytest.mark.parametrize('heading',[0.,3.10,-2.9])
def test_complete_kinematic_trajectory(kind,heading):
    c=MotionConfig();pose=Pose(1.,-1.,heading,1.);start=pose;e=MotionExecutor(kind,pose,c)
    path=[];signs=[];done=False
    for i in range(6000):
        command=e.sample(pose)
        assert not command.failure,command.failure
        assert abs(command.linear)<=c.linear_speed+1e-9 and abs(command.angular)<=c.angular_speed+1e-9
        if command.done:done=True;break
        s=1 if command.linear>.001 else -1 if command.linear<-.001 else 0
        if s and (not signs or s!=signs[-1]):signs.append(s)
        dt=.025
        pose=Pose(pose.x+command.linear*math.cos(pose.yaw)*dt,
                  pose.y+command.linear*math.sin(pose.yaw)*dt,
                  math.atan2(math.sin(pose.yaw+command.angular*dt),math.cos(pose.yaw+command.angular*dt)),pose.stamp+dt)
        path.append(pose)
    assert done,'Finite behavior did not complete'
    assert math.hypot(pose.x-start.x,pose.y-start.y)<.10
    if kind=='back_and_forth':
        assert e.completed_cycles==4 and signs==[1,-1]*4
    elif kind=='circle':
        assert e.swept>=2*math.pi-.026
        assert max(math.hypot(p.x-start.x,p.y-start.y) for p in path)>.8
    else:
        assert abs(math.atan2(math.sin(pose.yaw-heading),math.cos(pose.yaw-heading)))<.026


def test_stalled_and_deviating_robot_fail_closed():
    p=Pose(0,0,0,1);e=MotionExecutor('circle',p)
    assert e.sample(replace(p,stamp=6)).failure=='motion_stalled'
    e=MotionExecutor('back_and_forth',p)
    assert e.sample(Pose(2,0,0,2)).failure=='trajectory_deviation'


@pytest.mark.parametrize('kw',[{'linear_speed':math.nan},{'linear_speed':.6},{'cycles':0},{'circle_radius':3.}])
def test_invalid_motion_configuration(kw):
    with pytest.raises(ValueError):MotionConfig(**kw)


@pytest.mark.parametrize('change',[{'label':'bad'},{'confidence':math.nan},{'sequence':True},{'captured_unix':90.},
                                   {'captured_unix':101.},{'stable':1},{'session_id':'x'}])
def test_wire_protocol_rejects_invalid_or_stale_data(change):
    d=json.loads(observation().dumps());d.update(change)
    with pytest.raises(ValueError):Observation.loads(json.dumps(d),100.)


def test_wire_protocol_round_trip():assert Observation.loads(observation().dumps(),100.1)==observation()


def test_temporal_filter_rejects_jitter_freeze_and_gaps():
    f=StableGesture()
    assert not f.update('Victory',1.)
    assert not f.update('Pointing_Up',1.1)
    assert not f.update('Victory',1.2)
    for t in [1.3,1.4,1.5]:ok=f.update('Victory',t)
    assert ok
    assert not f.update('Victory',1.5)
    assert not f.update('Victory',2.5)


@pytest.mark.parametrize('mode',['static','linear','jitter','vertical'])
def test_wave_does_not_trigger_from_static_pose_or_camera_noise(mode):
    d=WaveDetector();detected=False
    for i in range(40):
        t=i*.06;x=.5 if mode in ('static','vertical') else .2+i*.009 if mode=='linear' else .5+.003*math.sin(i)
        y=.5+.15*math.sin(i*.7) if mode=='vertical' else .5
        detected|=d.update(t,x,y,.15)
    assert not detected


def test_wave_detects_lateral_reversal_and_expires():
    d=WaveDetector();detected=False
    for i in range(40):detected|=d.update(i*.06,.5+.12*math.sin(i*.55),.5,.15)
    assert detected
    assert not d.update(6.,.5,.5,.15)


def landmarks(extended=(True,True,True,True)):
    p=[(.5,.9,0.)]*21
    p[1],p[2],p[3],p[4]=(.2,.8,0.),(.22,.65,0.),(.22,.45,0.),(.22,.25,0.)
    for k,(mcp,x) in enumerate(zip((5,9,13,17),(.3,.42,.56,.68))):
        p[mcp]=(x,.65,0.);p[mcp+1]=(x,.5,0.)
        p[mcp+2]=(x,.35 if extended[k] else .62,0.);p[mcp+3]=(x,.2 if extended[k] else .75,0.)
    return p


@pytest.mark.parametrize('label,mask',[('Open_Palm',(1,1,1,1)),('Pointing_Up',(1,0,0,0)),('Victory',(1,1,0,0)),('Thumb_Up',(0,0,0,0)),('Closed_Fist',(0,0,0,0))])
def test_geometry_cross_checks_five_classes(label,mask):assert geometry_matches(label,landmarks(mask))


def test_policy_never_relabels_an_inconsistent_prediction():
    p=RecognitionPolicy();d=p.decide([([('Pointing_Up',.99),('Open_Palm',.01)],landmarks())],1.)
    assert d['label']=='None' and not d['valid_control']


def test_policy_rejects_ambiguous_two_hands():
    p=RecognitionPolicy();hand=([('Victory',.99)],landmarks((1,1,0,0)))
    d=p.decide([hand,hand],1.)
    assert d['reason']=='multiple_hands' and not d['valid_control']


def test_open_palm_stop_has_priority_with_two_hands_and_low_score():
    p=RecognitionPolicy();hand=([('Open_Palm',.56)],landmarks())
    d=p.decide([hand,hand],1.);assert d['label']=='Open_Palm' and d['stable']


def test_policy_rejects_low_confidence_and_margin():
    p=RecognitionPolicy();points=landmarks((1,1,0,0))
    assert not p.decide([([('Victory',.70)],points)],1.)['valid_control']
    assert not p.decide([([('Victory',.82),('Pointing_Up',.80)],points)],2.)['valid_control']


def test_scan_uses_full_surround_and_rejects_missing_coverage():
    ranges=[3.]*360;ranges[180]=.32
    assert scan_clearance(ranges,.12,10.,math.pi/180)==.32
    assert scan_clearance([math.inf]*360,.12,10.,math.pi/180)==10.
    assert scan_clearance([math.nan]*360,.12,10.,math.pi/180) is None
    assert scan_clearance([3.]*90,.12,10.,math.pi/180) is None


def test_policy_emits_wave_only_after_real_temporal_palm_motion():
    policy=RecognitionPolicy();seen=[]
    for i in range(55):
        shift=.23*math.sin(i*.5)
        points=[(x+shift,y,z) for x,y,z in landmarks()]
        seen.append(policy.decide([([('Open_Palm',.99)],points)],i*.06))
    assert any(d['label']=='Wave' and d['stable'] for d in seen)
    policy.reset()
    d=policy.decide([([('Open_Palm',.99)],landmarks())],5.)
    assert d['label']=='Open_Palm' and d['stable']


def test_single_below_range_return_cannot_hide_a_close_obstacle():
    ranges=[3.]*360;ranges[10]=.03
    assert scan_clearance(ranges,.12,10.,math.pi/180)==0.



def test_wave_starts_on_first_recognized_frame_and_does_not_retrigger():
    m=BehaviorMachine()
    m.observe(observation('Wave',1,raw_label='Open_Palm'),0.,Pose(0,0,0,0.))
    assert m.state=='RUNNING' and m.executor.kind=='greeting'
    executor=m.executor
    feed(m,'Wave',.05,seq=1,raw_label='Open_Palm')
    assert m.executor is executor
    m.executor.finished=True;m.tick(.7,Pose(0,0,0,.7))
    feed(m,'Wave',.75,seq=20,raw_label='Open_Palm')
    assert m.state=='IDLE' and m.completions==1


def test_new_wave_after_hand_release_starts_a_new_greeting():
    m=BehaviorMachine();t,seq=feed(m,'Wave',raw_label='Open_Palm')
    m.executor.finished=True;m.tick(t,Pose(0,0,0,t))
    t,seq=feed(m,'None',t,seq=seq,hand_visible=False,valid_control=False)
    m.observe(observation('Wave',seq+1,raw_label='Open_Palm'),t,Pose(0,0,0,t))
    assert m.state=='RUNNING' and m.completions==1


def test_wave_respects_sensor_start_guards():
    m=BehaviorMachine()
    m.observe(observation('Wave',1,raw_label='Open_Palm'),0.,Pose(0,0,0,0.),'obstacle_too_close')
    assert m.state=='IDLE' and m.reason=='obstacle_too_close'


def test_real_temporal_wave_starts_behavior_without_static_hold():
    policy=RecognitionPolicy();m=BehaviorMachine();first=None
    # A single broad out-and-back wave, sampled at 10 FPS.
    shifts=[0.,.10,.20,.30,.20,.10,0.]
    for i,shift in enumerate(shifts):
        points=[(x+shift,y,z) for x,y,z in landmarks()]
        decision=policy.decide([([('Open_Palm',.99)],points)],i*.1)
        o=Observation(session_id='integration-wave',sequence=i,captured_unix=100.+i*.1,**decision)
        m.observe(o,i*.1,Pose(0,0,0,i*.1))
        if m.state=='RUNNING' and first is None:first=i*.1
    assert first is not None and first<=.60
    assert m.selected=='greeting'


def test_greeting_is_visibly_fast_at_new_speed():
    e=MotionExecutor('greeting',Pose(0,0,0,0.));p=Pose(0,0,0,0.)
    peak=0.
    for _ in range(600):
        c=e.sample(p);assert not c.failure
        peak=max(peak,abs(c.angular))
        if c.done:break
        p=Pose(0,0,wrap_yaw(p.yaw+c.angular*.025),p.stamp+.025)
    assert e.finished and p.stamp<4.5 and peak==pytest.approx(1.10)


def wrap_yaw(a):
    return math.atan2(math.sin(a),math.cos(a))
