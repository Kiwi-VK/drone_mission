#!/usr/bin/env python3
from __future__ import annotations
from dataclasses import dataclass, field
MISSION_SCORE_MAX=90
@dataclass(frozen=True)
class DropEvent: point_id:int; expected_color:str; detected_color:str; released:bool; inside_target_ring:bool
@dataclass(frozen=True)
class TargetEvent: detected:bool; obstacle_id:int|None; target_height_m:float|None; descent_cm:float; altitude_held_after_descent:bool
@dataclass(frozen=True)
class LandingEvent: airframe_span_m:float; in_landing_zone:bool; fully_inside:bool; visual_closed_loop:bool
@dataclass(frozen=True)
class ObstacleEvent: depth_sensor_used:bool; route_replanned_from_observation:bool; returned_to_route:bool; emergency_stop_available:bool; prewritten_route_used:bool=False
@dataclass(frozen=True)
class CompetitionScenario: drop_events:tuple[DropEvent,...]; target_event:TargetEvent; landing_event:LandingEvent; obstacle_event:ObstacleEvent; collision:bool=False; entered_end_zone_early:bool=False; manual_stop:bool=False
@dataclass
class AcceptanceResult: passed:bool; errors:list[str]=field(default_factory=list); score:int=0
def evaluate_scenario(s):
    errors=[]; score=0
    if tuple(e.point_id for e in s.drop_events)!=(1,2,3,4): errors.append('投放顺序必须严格 1-2-3-4')
    if len(s.drop_events)!=4: errors.append('必须完成 4 个投放点')
    for e in s.drop_events:
        if e.expected_color==e.detected_color and e.released and e.inside_target_ring: score+=15
        else: errors.append(f'投放点 {e.point_id} 未正确投放')
    t=s.target_event
    if not t.detected or t.obstacle_id not in (1,2) or (t.target_height_m or 0)<1.2 or abs(t.descent_cm-20)>2: errors.append('目标识别/下降 20cm 不合规')
    elif not t.altitude_held_after_descent: errors.append('下降 20cm 后未保持后续飞行高度')
    else: score+=20
    o=s.obstacle_event
    if not o.depth_sensor_used: errors.append('随机障碍物必须使用真实深度/测距硬件')
    if not o.route_replanned_from_observation or not o.returned_to_route: errors.append('必须实时避障并恢复原航线')
    if not o.emergency_stop_available: errors.append('必须有碰撞兜底悬停')
    if o.prewritten_route_used: errors.append('不得预写死 A/B 绕行路线')
    l=s.landing_event
    if l.in_landing_zone and l.fully_inside and l.visual_closed_loop and l.airframe_span_m<=0.60: score+=10
    else: errors.append('降落区视觉闭环/机架全部进入不合规')
    if s.collision: errors.append('碰撞障碍物')
    if s.entered_end_zone_early: errors.append('提前进入终点区')
    if s.manual_stop and score<MISSION_SCORE_MAX: errors.append('人工停止时任务未完成')
    return AcceptanceResult(not errors,errors,score)
def compliant_demo():
    colors=('black','white','red','yellow')
    return CompetitionScenario(tuple(DropEvent(i,c,c,True,True) for i,c in enumerate(colors,1)),TargetEvent(True,2,1.3,20,True),LandingEvent(0.45,True,True,True),ObstacleEvent(True,True,True,True,False))
def run_acceptance_suite():
    r=evaluate_scenario(compliant_demo())
    if not r.passed or r.score!=MISSION_SCORE_MAX: print('FAIL compliant',r); return 1
    bad=compliant_demo(); r=evaluate_scenario(CompetitionScenario(bad.drop_events,bad.target_event,bad.landing_event,ObstacleEvent(False,False,False,False,True)))
    if r.passed: print('FAIL obstacle negative'); return 1
    print(f'PASS image-rule acceptance suite; mission score max={MISSION_SCORE_MAX}'); return 0
if __name__=='__main__': raise SystemExit(run_acceptance_suite())
