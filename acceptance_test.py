"""Regression and failure injection tests; independent of Pi hardware and third party deps."""
import json, math, tempfile, time, unittest
from pathlib import Path
from unittest.mock import patch
import config as C
from serial_comm import FlightController, MockFlightController, FlightError, Location
from mission import MissionController
from vision import MockVisionProcessor
from obstacle_avoidance import OccupancyGrid, LocalObstaclePlanner, MockGridReceiver
from payload import MockPayloadFeedback
from visual_servo import PixelPID

class FakeConstants:
    MAV_MODE_FLAG_SAFETY_ARMED=128
    MAV_AUTOPILOT_ARDUPILOTMEGA=3
class FakeMessage:
    def __init__(self,kind,**values): self.kind=kind; self.__dict__.update(values)
    def get_type(self): return self.kind
    def get_srcSystem(self): return getattr(self,'system',1)
    def get_srcComponent(self): return 1
class RegressionTests(unittest.TestCase):
    def controller(self,**kw):
        c=MissionController(simulation=True,**kw)
        self.addCleanup(c.close); return c
    def test_real_heartbeat_constants_and_source_filter(self):
        import threading
        from collections import deque
        from types import SimpleNamespace
        f=FlightController.__new__(FlightController)
        f.m=FakeConstants; f.mu=SimpleNamespace(mode_string_v10=lambda m:'GUIDED')
        f.target_system=f.target_component=1; f._lock=threading.RLock(); f._seen={}; f._acks=deque()
        f._consume(FakeMessage('HEARTBEAT',base_mode=128,autopilot=3,type=2))
        self.assertTrue(f._armed); self.assertEqual(f._mode,'GUIDED')
        f._consume(FakeMessage('HEARTBEAT',base_mode=0,autopilot=3,type=2,system=7))
        self.assertTrue(f._armed)
    def test_battery_voltage_is_pack_not_first_cell(self):
        import threading
        f=FlightController.__new__(FlightController); f.target_system=f.target_component=1; f._lock=threading.RLock(); f._seen={}
        f._consume(FakeMessage('SYS_STATUS',voltage_battery=22000))
        self.assertEqual(f._battery,22)
    def test_firmware_gate_and_stale_telemetry(self):
        f=FlightController.__new__(FlightController); f.m=FakeConstants
        f._reader_error=None; f._autopilot=12; f._vehicle=2; f._seen={}; f._ekf=0; f._battery=0; f._down_range=None
        errors=f.health_errors()
        self.assertTrue(any('ArduCopter' in e for e in errors)); self.assertTrue(any('stale' in e for e in errors))
    def test_frozen_20cm_descent_cannot_pass(self):
        c=self.controller(); c.state='NAVIGATE'; c.fc._location=Location(0,0,1.3); c.fc.frozen=True
        for _ in range(20): c.cycle()
        self.assertIsNotNone(c.target_start); self.assertFalse(c.target_observed)
        self.assertAlmostEqual(c.active_altitude,1.1)
    def test_descent_verified_and_persisted(self):
        c=self.controller(); c.state='NAVIGATE'; c.fc._location=Location(0,0,1.3)
        for _ in range(220):
            c.cycle()
            if c.target_observed: break
        self.assertTrue(c.target_observed); self.assertAlmostEqual(c.active_altitude,1.1)
        self.assertLess(abs(c.fc.get_current_location().alt-1.1),C.DESCENT_TOLERANCE_M)
    def test_camera_loss_aborts(self):
        v=MockVisionProcessor(); v.dropout=True; c=self.controller(vision=v)
        with patch('mission.time.monotonic',side_effect=[0,10]):
            # Test immediate safety cycle rather than camera startup grace period.
            pass
        with self.assertRaises(FlightError): c.cycle()
    def test_bad_depth_is_not_clear(self):
        g=MockGridReceiver(); g.stale=True; c=self.controller(grid=g)
        c.fc._location=Location(0,0,1.3)
        with self.assertRaises(FlightError): c.cycle()
    def test_stop_response_lands_and_never_rtl(self):
        c=self.controller(); c.fc.arm(); c.airborne=True; c.request_stop()
        self.assertFalse(c.run()); self.assertIn('LAND',c.fc.mode_log); self.assertNotIn('RTL',c.fc.mode_log)
    def test_manual_takeover_is_respected(self):
        c=self.controller(); c.fc.arm(); c.airborne=True; c.fc._mode='LOITER'
        c.abort('pilot takeover'); self.assertNotIn('LAND',c.fc.mode_log)
    def test_low_battery_in_every_cycle(self):
        c=self.controller(); c.fc.fault='Battery low'
        with self.assertRaisesRegex(FlightError,'Battery low'): c.cycle()
    def test_terminal_land_requires_disarm(self):
        f=MockFlightController(); f.arm(); f.begin_land(); f._location=Location(0,0,0.4)
        self.assertFalse(f.landed_confirmed())
        f._location=Location(0,0,0); self.assertFalse(f.landed_confirmed())
        f._armed=False; self.assertTrue(f.landed_confirmed())
    def test_successful_mission_has_measured_evidence_no_rtl(self):
        c=self.controller(); self.assertTrue(c.run()); c.close()
        self.assertEqual(c.completed_drops,4); self.assertEqual(c.fc.mode_log,['GUIDED','LAND'])
        events=[json.loads(s) for s in c.evidence.path.read_text(encoding='utf-8').splitlines()]
        self.assertTrue(all(e['simulation'] for e in events))
        descent=next(e for e in events if e['event']=='target_descent_verified')
        self.assertAlmostEqual(descent['measured_descent_m'],0.2,delta=C.DESCENT_TOLERANCE_M)
        self.assertIsNone(events[-1]['competition_score'])
    def test_jammed_servo_fails_no_retry(self):
        p=MockPayloadFeedback(); p.jammed=True; c=self.controller(payload=p)
        c.fc._location=Location(0,0,1.3)
        with self.assertRaises(FlightError): c.release(0)
        self.assertEqual(c.completed_drops,0)
        self.assertEqual(len(c.fc.servo_log),2)
    def test_pid_time_and_axes(self):
        p=PixelPID(10,matrix=[[0,-1],[1,0]])
        first=p.update((400,240),(480,640,3),now=1.)
        self.assertGreater(first.vy,0); self.assertEqual(first.vx,0)
        second=p.update((400,240),(480,640,3),now=1.1)
        self.assertLess(abs(second.vy-first.vy),0.01)
    def test_unknown_grid_cannot_route(self):
        g=OccupancyGrid(1,time.monotonic(),.1,(-3.,-3.),60,60,tuple([-1]*3600))
        with self.assertRaises(FlightError): LocalObstaclePlanner(simulation=True).plan(g,(0,0),(1,0))
    def test_astar_detour_is_from_observation(self):
        receiver=MockGridReceiver(); planner=LocalObstaclePlanner(simulation=True)
        grid=receiver.read(); path=planner.plan(grid,(0,0),(2,0))
        self.assertEqual(planner.state,'AVOID')
        self.assertTrue(any(abs(e)>0.25 for _,e in path))
        self.assertTrue(all(not planner.blocked(grid,grid.index(*p)) for p in path))
        receiver.obstacles=[]; path=planner.plan(receiver.read(),(0,0),(2,0))
        self.assertEqual(planner.state,'NAVIGATE'); self.assertEqual(path,[(2,0)])
    def test_delayed_grid_rejected(self):
        data={'sequence':1,'sample_monotonic_s':1,'width':3,'height':3,'resolution_m':.1,'origin_ne':[0,0],'cells':[0]*9,'frame':'takeoff_NED','coverage':'full_airframe_height'}
        with self.assertRaises(ValueError): OccupancyGrid.parse(json.dumps(data),received_at=2)
        data['sample_monotonic_s']=1.9
        self.assertEqual(OccupancyGrid.parse(json.dumps(data),received_at=2).sequence,1)
    def test_unconfigured_real_run_is_blocked(self):
        errors=C.validate_config(simulation=False)
        self.assertGreater(len(errors),10)

if __name__=='__main__': unittest.main(verbosity=2)
