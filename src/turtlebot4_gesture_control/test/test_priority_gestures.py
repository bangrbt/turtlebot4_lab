"""Regressions for real perception flicker, priority cancellation and wave-to-stop transitions."""
import math
import pytest
from turtlebot4_gesture_control.behavior import BehaviorMachine
from turtlebot4_gesture_control.gestures import Observation, RecognitionPolicy, hand_shape
from turtlebot4_gesture_control.motion import Pose


def hand(extended=(1,1,1,1), thumb_tucked=False):
    p=[(.5,.9,0.)]*21
    p[1],p[2],p[3],p[4]=(.2,.8,0.),(.22,.65,0.),(.22,.45,0.),(.22,.25,0.)
    if thumb_tucked:
        p[2],p[3],p[4]=(.25,.73,0.),(.28,.63,0.),(.40,.66,0.)
    for opened,(mcp,x) in zip(extended,zip((5,9,13,17),(.3,.42,.56,.68))):
        p[mcp]=(x,.65,0.);p[mcp+1]=(x,.5,0.)
        p[mcp+2]=(x,.35 if opened else .62,0.);p[mcp+3]=(x,.2 if opened else .75,0.)
    return p


def obs(label='Victory',seq=1,**kw):
    d=dict(label=label,confidence=.99,raw_label=label,raw_confidence=.99,
           stable=True,valid_control=True,hand_visible=True,session_id='priority-test',
           sequence=seq,captured_unix=100.,reason='test')
    d.update(kw)
    return Observation(**d)


def running(label):
    m=BehaviorMachine();sequence=0;t=0.
    for name in ([label] if label=='Wave' else [label,'Thumb_Up']):
        for _ in range(12):
            sequence+=1;t+=.05
            m.observe(obs(name,sequence),t,Pose(0,0,0,t))
    assert m.state=='RUNNING'
    return m,t,sequence


@pytest.mark.parametrize('selector',['Wave','Pointing_Up','Victory'])
@pytest.mark.parametrize('score',[.50,.55,.60,.64])
def test_raw_fist_cancels_every_behavior_on_the_first_frame(selector,score):
    m,t,seq=running(selector)
    m.observe(obs('None',seq+1,confidence=score,raw_label='Closed_Fist',raw_confidence=score,
                  valid_control=False,stable=False),t+.01,Pose(0,0,0,t+.01))
    assert m.state=='IDLE' and m.selected is None and m.executor is None
    assert m.reason=='closed_fist_cancel'
    assert m.tick(t+.02,Pose(0,0,0,t+.02)).linear==0.


def test_accepted_landmark_cancel_does_not_get_rejected_by_a_second_confidence_gate():
    m,t,seq=running('Victory')
    m.observe(obs('Closed_Fist',seq+1,confidence=0.,raw_label='None',raw_confidence=.91,
                  reason='fist_landmarks_cancel'),t+.01,Pose(0,0,0,t+.01))
    assert m.state=='IDLE' and m.reason=='closed_fist_cancel'


def test_fist_policy_does_not_wait_for_static_gesture_stability():
    policy=RecognitionPolicy()
    d=policy.decide([([('Closed_Fist',.54)],hand((0,0,0,0),True))],0.)
    assert d['label']=='Closed_Fist' and d['valid_control'] and d['stable']


def test_cancel_has_priority_over_another_hand_open_palm():
    policy=RecognitionPolicy()
    d=policy.decide([([('Open_Palm',.99)],hand()),
                     ([('Closed_Fist',.54)],hand((0,0,0,0),True))],0.)
    assert d['label']=='Closed_Fist' and d['reason']=='cancel_priority'


def test_landmark_fist_fallback_survives_classifier_none_without_inventing_a_score():
    policy=RecognitionPolicy();pts=hand((0,0,0,0),True)
    assert hand_shape(pts)[1]
    first=policy.decide([([('None',.85)],pts)],0.)
    assert first['label']!='Closed_Fist'
    d=policy.decide([([('None',.82)],pts)],.1)
    assert d['label']=='Closed_Fist' and d['valid_control'] and d['reason']=='fist_landmarks_cancel'
    assert d['raw_label']=='None' and d['raw_confidence']==.82 and d['confidence']==0.


@pytest.mark.parametrize('label,extended',[('Thumb_Up',(0,0,0,0)),('Pointing_Up',(1,0,0,0)),('Victory',(1,1,0,0)),('Open_Palm',(1,1,1,1))])
def test_other_required_gestures_never_trigger_landmark_fist_fallback(label,extended):
    policy=RecognitionPolicy();pts=hand(extended)
    assert not hand_shape(pts)[1]
    for i in range(10):
        d=policy.decide([([(label,.99)],pts)],i*.1)
        assert d['label']!='Closed_Fist'


@pytest.mark.parametrize('rotation',[0.,.8,1.5,-1.5])
def test_fist_geometry_is_independent_of_in_plane_hand_orientation(rotation):
    pts=[(x*math.cos(rotation)-y*math.sin(rotation),x*math.sin(rotation)+y*math.cos(rotation),z)
         for x,y,z in hand((0,0,0,0),True)]
    assert hand_shape(pts)[1]


def wave_sequence(flicker=False, wrists_fixed=False):
    policy=RecognitionPolicy();machine=BehaviorMachine();outputs=[];started=None
    for i,shift in enumerate([0.,.08,.16,.24,.16,.08,0.,-.08,-.16,-.08,0.]):
        points=hand()
        indices=(8,12,16,20) if wrists_fixed else range(21)
        points=[(x+(shift if j in indices else 0.),y,z) for j,(x,y,z) in enumerate(points)]
        raw_label='Open_Palm' if not flicker or i%2==0 else 'None'
        score=.75 if raw_label=='Open_Palm' else .9
        d=policy.decide([([(raw_label,score)],points)],i*.1)
        outputs.append(d)
        machine.observe(Observation(session_id='wave-integration',sequence=i,captured_unix=100.+i*.1,**d),i*.1,Pose(0,0,0,i*.1))
        if machine.state=='RUNNING' and started is None:started=i*.1
        if started is not None:
            assert machine.state=='RUNNING', d
    return started,outputs


@pytest.mark.parametrize('flicker',[False,True])
def test_wave_with_classifier_flicker_starts_early_and_stays_running(flicker):
    started,decisions=wave_sequence(flicker=flicker)
    assert started is not None and started<=.5
    assert any(d['label']=='Wave' for d in decisions)


def test_wave_detects_fingertip_motion_while_wrist_is_nearly_stationary():
    started,_=wave_sequence(wrists_fixed=True)
    assert started is not None and started<=.6


def test_brief_no_hand_frame_does_not_erase_an_in_progress_wave():
    policy=RecognitionPolicy();seen=False
    for i,shift in enumerate([0.,.1,.2,.3,.2,.1,0.]):
        points=[(x+shift,y,z) for x,y,z in hand()]
        d=policy.decide([] if i==3 else [([('Open_Palm',.8)],points)],i*.1)
        seen|=d['label']=='Wave'
    assert seen


def test_moving_palm_or_pose_flicker_cannot_interrupt_its_own_greeting():
    m,t,seq=running('Wave')
    for i,reason in enumerate(['palm_motion','palm_pose_uncertain']):
        m.observe(obs('Open_Palm',seq+i+1,raw_label='Open_Palm',reason=reason),t+.05*(i+1),Pose(0,0,0,t))
        assert m.state=='RUNNING'
    m.observe(obs('Open_Palm',seq+3,reason='palm_stationary'),t+.2,Pose(0,0,0,t))
    assert m.state=='IDLE' and m.reason=='open_palm_stop'


def test_wave_can_be_rearmed_after_stationary_palm_stop_without_hiding_the_hand():
    m,t,seq=running('Wave')
    m.observe(obs('Open_Palm',seq+1,reason='palm_stationary'),t+.1,Pose(0,0,0,t))
    m.observe(obs('Wave',seq+2,raw_label='None',confidence=0.,reason='wave_landmarks'),t+.2,Pose(0,0,0,t))
    assert m.state=='RUNNING' and m.selected=='greeting'


def test_long_tracking_gap_cannot_join_two_unrelated_hand_movements():
    policy=RecognitionPolicy()
    for i,shift in enumerate([0.,.1,.2]):
        pts=[(x+shift,y,z) for x,y,z in hand()]
        policy.decide([([('Open_Palm',.8)],pts)],i*.1)
    policy.decide([],1.)
    d=policy.decide([([('Open_Palm',.8)],hand())],1.1)
    assert d['label']!='Wave'
