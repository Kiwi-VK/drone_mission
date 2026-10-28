"""Mission state machine. One velocity per cycle; all phases share safety arbitration."""
from __future__ import annotations
import argparse, json, logging, math, signal, threading, time, uuid
from pathlib import Path
import config as C
from serial_comm import FlightError, clamp, get_flight_controller
from vision import VisionProcessor, MockVisionProcessor
from visual_servo import PixelPID
from obstacle_avoidance import GridReceiver, MockGridReceiver, LocalObstaclePlanner
from payload import PayloadFeedback, MockPayloadFeedback
log=logging.getLogger('mission')
class Evidence:
    def __init__(self,simulation):
        C.LOG_DIR.mkdir(parents=True,exist_ok=True)
        self.path=C.LOG_DIR/(('simulation-' if simulation else 'hardware-')+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]+'.jsonl')
        self.simulation=simulation
    def write(self,event,**data):
        with self.path.open('a',encoding='utf-8') as f:
            f.write(json.dumps({'event':event,'monotonic_s':time.monotonic(),'simulation':self.simulation,**data},ensure_ascii=False,allow_nan=False)+'\n')

class MissionController:
    def __init__(self,*,simulation,port=None,baud=None,fc=None,vision=None,grid=None,payload=None):
        self.simulation=simulation; self.fc=fc; self.vision=vision; self.grid=grid
        self.payload=payload or (MockPayloadFeedback() if simulation else PayloadFeedback())
        self.planner=LocalObstaclePlanner(simulation=simulation)
        self.stop_requested=threading.Event(); self.watchdog_stop=threading.Event(); self._abort_lock=threading.Lock()
        self.aborted=False; self.failure=None; self.airborne=False; self.completed=False; self.state='INIT'
        self.target_observed=False; self.target_candidate=None; self.target_count=0; self.target_start=None
        self.target_last_seq=None; self.active_altitude=C.TAKEOFF_ALTITUDE
        self.start_time=None; self.last_cycle=time.monotonic(); self.completed_drops=0; self.final_approach=False
        self.evidence=Evidence(simulation); self.watchdog=None
        try:
            if fc is None: self.fc=get_flight_controller(simulation=simulation,port=port,baud=baud)
            if vision is None: self.vision=MockVisionProcessor() if simulation else VisionProcessor()
            if grid is None: self.grid=MockGridReceiver() if simulation else GridReceiver()
            self.fc.cancel=self.stop_requested.is_set; self.fc.guard=self._guard
        except BaseException: self.close(); raise
    def request_stop(self,*_): self.stop_requested.set()
    def _guard(self):
        if self.aborted: raise FlightError(self.failure or 'Aborted')
        if self.stop_requested.is_set(): raise FlightError('Stop requested')
        if self.start_time is not None and time.monotonic()-self.start_time>C.MAX_FLIGHT_TIME: raise FlightError('Mission time exceeded')
        errors=self.fc.health_errors(airborne=self.airborne and self.state!='LAND')
        if errors: raise FlightError('; '.join(errors))
        p=self.fc.get_current_location()
        if p and not self.simulation:
            if not C.inside_field(p.north,p.east): raise FlightError('Geofence breach')
            if p.alt>C.MAX_ALTITUDE: raise FlightError('Altitude ceiling exceeded')
            if self.airborne and self.state not in ('TAKEOFF','LAND') and p.alt<0.18: raise FlightError('Unexpected low altitude')
            end=C.FIELD.get('end_zone')
            r=C.FRAME_SPAN_M/2+C.POSITION_MARGIN_M
            if not self.final_approach and end and end[0]-r<=p.north<=end[1]+r and end[2]-r<=p.east<=end[3]+r:
                raise FlightError('End-zone entry before all tasks complete')
    def _watch(self):
        while not self.watchdog_stop.wait(0.05):
            if not self.airborne or self.completed: continue
            try:
                self._guard()
                if time.monotonic()-self.last_cycle>0.8: raise FlightError('Control loop stalled')
            except Exception as exc: self.abort(str(exc)); return
    def abort(self,reason):
        with self._abort_lock:
            if self.aborted: return
            self.aborted=True; self.failure=reason
            log.error('ABORT: %s',reason)
            requested=False
            try:
                if self.fc and self.fc.is_armed(): requested=self.fc.emergency_land()
            except Exception as exc: log.error('FC safety command unavailable: %s',exc)
            self.evidence.write('abort',reason=reason,land_requested=requested,landed_confirmed=bool(self.fc and self.fc.landed_confirmed()))
            # If link lost or pilot took over, onboard FC failsafe/manual control is authoritative.
    def preflight(self):
        errors=C.validate_config(simulation=self.simulation)
        if errors: raise FlightError('\n'.join(errors))
        end=time.monotonic()+5
        while True:
            self.last_cycle=time.monotonic()
            try:
                self._guard(); self.grid.read(); self.vision.read_bottom_frame(); self.vision.read_front_frame()
                if not self.simulation:
                    p=self.fc.get_current_location()
                    if p is None or math.hypot(p.north,p.east)>0.15 or abs(p.alt)>0.10: raise FlightError('LOCAL_POSITION_NED origin does not match static takeoff point')
                    if self.fc.is_armed(): raise FlightError('Already armed: preflight refuses')
                for i,s in enumerate(C.PACKAGE_SLOTS):
                    if self.payload.read(i+1,s['color'],present=True) is None: raise FlightError('Load/color feedback mismatch')
                break
            except FlightError:
                if time.monotonic()>end or self.stop_requested.is_set(): raise
                self.fc.pause(0.05,check=False)
        self.evidence.write('preflight',ready=True)
    def _target_tick(self):
        sample=self.vision.read_front_frame()
        if self.target_observed or self.state in ('TAKEOFF','LAND'): return
        if sample.sequence==self.target_last_seq: return
        self.target_last_seq=sample.sequence
        detection=self.vision.detect_target_image(sample.frame)
        if not detection:
            self.target_count=0; return
        oid=detection.get('obstacle_id')
        if oid not in (1,2): raise FlightError('Target obstacle identity invalid')
        if not self.simulation:
            p=self.fc.get_current_location(); obstacle=C.MISSION_DATA['fixed_obstacles'][str(oid)]
            bearing=math.atan2(obstacle['east_m']-p.east,obstacle['north_m']-p.north)-p.yaw
            expected=(bearing+math.pi)%(2*math.pi)-math.pi
            image_bearing=math.radians((detection['center'][0]/sample.frame.shape[1]-0.5)*C.FRONT_HFOV_DEG)
            if abs(expected-image_bearing)>math.radians(12): self.target_count=0; return
        self.target_count=self.target_count+1 if oid==self.target_candidate else 1
        self.target_candidate=oid
        if self.target_count>=C.TARGET_CONFIRM_FRAMES and self.target_start is None:
            self.target_start=self.fc.get_current_location().alt
            target=self.target_start-0.2
            if target<C.MIN_ALLOWED_ALTITUDE: raise FlightError('Insufficient height for exact 20cm descent')
            self.active_altitude=target; self.descent_start_time=time.monotonic(); self.descent_stable=0
            self.evidence.write('target_recognized',obstacle_id=oid,start_alt_m=self.target_start,target_alt_m=target,target_installation_height_m=None)
    def _descent_velocity(self):
        if self.target_start is None or self.target_observed: return None
        p=self.fc.get_current_location(); delta=self.target_start-p.alt
        if time.monotonic()-self.descent_start_time>10: raise FlightError('20cm descent timed out')
        if abs(delta-0.2)<=C.DESCENT_TOLERANCE_M:
            self.descent_stable+=1
            if self.descent_stable>=C.VISUAL_STABLE_FRAMES:
                self.target_observed=True
                self.evidence.write('target_descent_verified',measured_descent_m=delta,active_altitude_m=self.active_altitude)
                return (0.,0.,0.)
        else: self.descent_stable=0
        return (0.,0.,clamp((p.alt-self.active_altitude)*1.5,C.MAX_VERTICAL_SPEED_MPS))
    def cycle(self,north=0.,east=0.,down=None,*,inspect_target=True):
        self.last_cycle=time.monotonic(); self._guard()
        if inspect_target: self._target_tick()
        p=self.fc.get_current_location()
        if p is None: raise FlightError('No current position')
        descent=self._descent_velocity()
        if descent is not None: north,east,down=descent
        elif down is None: down=clamp((p.alt-self.active_altitude)*1.2,C.MAX_VERTICAL_SPEED_MPS)
        g=self.grid.read()
        self.planner.arbitrate(g,(p.north,p.east),(north,east),allow_end=self.final_approach)
        self._guard()  # no movement after watchdog abort/pilot takeover
        self.fc.send_velocity(north,east,down)
        self.fc.pause(C.CONTROL_PERIOD_S)
        return descent is None
    def navigate(self,north,east):
        self.state='NAVIGATE'; end=time.monotonic()+C.MISSION_NAV_TIMEOUT_S; steps=0; previous=None
        while time.monotonic()<end and steps<1200:
            steps+=1; self._guard(); p=self.fc.get_current_location(); g=self.grid.read()
            path=self.planner.plan(g,(p.north,p.east),(north,east),allow_end=self.final_approach)
            if self.planner.state!=previous:
                self.evidence.write('navigation_state',state=self.planner.state,grid_sequence=g.sequence,path_ne=path)
                previous=self.planner.state
            distance=math.hypot(north-p.north,east-p.east)
            if distance<C.NAVIGATION_TOLERANCE_M:
                if self.cycle() and abs(p.alt-self.active_altitude)<C.ALTITUDE_TOLERANCE_M: return
                continue
            waypoint=path[0]
            if math.dist((p.north,p.east),waypoint)<0.08 and len(path)>1: waypoint=path[1]
            dn,de=waypoint[0]-p.north,waypoint[1]-p.east
            scale=min(0.8,C.MAX_AVOID_SPEED_MPS/max(math.hypot(dn,de),1e-9))
            self.cycle(dn*scale,de*scale)
        raise FlightError('Navigation timeout')
    def _pid(self,tolerance): return PixelPID(tolerance,matrix=[[0,-1],[1,0]] if self.simulation else None)
    def _visual_error_m(self,center,shape):
        h,w=shape[:2]; z=self.fc.get_down_range()
        fx,fy=(600.,600.) if self.simulation else (C.CAMERA_CALIBRATION['fx'],C.CAMERA_CALIBRATION['fy'])
        return math.hypot((center[0]-w/2)*z/fx,(center[1]-h/2)*z/fy)
    def align(self,point_id=None,*,landing=False):
        self.state='VISUAL_LAND' if landing else 'ALIGN'
        pid=self._pid(C.LANDING_CENTER_TOLERANCE_PX if landing else C.RING_CENTER_TOLERANCE_PX)
        stable=0; last_seq=None; end=time.monotonic()+(C.LANDING_ALIGN_TIMEOUT_S if landing else C.VISUAL_ALIGN_TIMEOUT_S); steps=0
        while time.monotonic()<end and steps<1000:
            steps+=1; self._guard(); sample=self.vision.read_bottom_frame()
            if sample.sequence==last_seq: self.cycle(); continue
            last_seq=sample.sequence
            detection=self.vision.detect_landing_zone(sample.frame) if landing else self.vision.detect_collection_point(sample.frame,point_id)
            if not detection or (not landing and detection.get('point_id')!=point_id):
                stable=0; pid.reset(); self.cycle(); continue
            cmd=pid.update(detection['center'],sample.frame.shape)
            error_m=self._visual_error_m(detection['center'],sample.frame.shape)
            # Landing metric includes complete gear footprint. Inner ring margin includes 5cm cube.
            tolerance_m=(0.60-C.LANDING_GEAR_SPAN_M)/2-C.LANDING_MARGIN_M if landing else 0.075-0.05*math.sqrt(2)/2-0.01
            centered=cmd.centered and error_m<=tolerance_m
            if centered:
                stable+=1
                if landing and stable>=C.VISUAL_STABLE_FRAMES:
                    if self.fc.get_down_range()<=C.LAND_HANDOVER_ALT_M:
                        self.evidence.write('landing_handover',range_m=self.fc.get_down_range(),visual_error_m=error_m)
                        return
                    if self.cycle(0,0,C.MAX_VERTICAL_SPEED_MPS): continue
                    stable=0
                elif not landing and stable>=C.VISUAL_STABLE_FRAMES:
                    if self.cycle(): return
                    stable=0
                self.cycle()
            else:
                stable=0; p=self.fc.get_current_location()
                n=cmd.vx*math.cos(p.yaw)-cmd.vy*math.sin(p.yaw)
                e=cmd.vx*math.sin(p.yaw)+cmd.vy*math.cos(p.yaw)
                self.cycle(n,e,0. if landing else None)
        raise FlightError('Visual alignment/descent timeout or target lost')
    def release(self,index):
        self.state='RELEASE'; s=C.PACKAGE_SLOTS[index]; color=C.DROP_SEQUENCE[index]; slot=index+1
        before=self.payload.read(slot,color,present=True)
        if before is None: raise FlightError('Loaded color/slot not verified')
        self.cycle(); self.fc.set_servo(s['channel'],s['open_pwm']); opened=time.monotonic()
        try:
            if self.simulation: self.payload.release(slot)
            count=0
            while time.monotonic()-opened<C.SERVO_ACTION_DURATION and count<8:
                self.cycle(); count+=1
            end=time.monotonic()+2; count=0
            while time.monotonic()<end and count<40:
                count+=1; self.cycle()
                after=self.payload.read(slot,color,present=False)
                if after is not None and after>before:
                    self.completed_drops+=1
                    self.evidence.write('payload_release_verified',point_id=slot,color=color,feedback_event=after,first_impact_ring=None)
                    return
            raise FlightError('Release unconfirmed/jammed: no blind retry')
        finally:
            # Closing attempts are bounded and bypass a latched abort, but never switch flight modes.
            try:
                if not self.simulation: self.fc.command(self.fc.m.MAV_CMD_DO_SET_SERVO,s['channel'],s['close_pwm'],timeout=0.5,check=False)
                else: self.fc.set_servo(s['channel'],s['close_pwm'])
            except Exception as exc: log.error('Servo close unconfirmed: %s',exc)
    def _point(self,p,index):
        if self.simulation: return ((0.7,0.7),(1.5,0.7),(1.5,-0.8),(0.5,-0.8),(0.,0.))[index]
        return p['north_m'],p['east_m']
    def run(self):
        try:
            self.preflight(); self.start_time=time.monotonic(); self.state='TAKEOFF'
            self.fc.arm(); self.airborne=True; self.last_cycle=time.monotonic()
            if not self.simulation:
                self.watchdog=threading.Thread(target=self._watch,daemon=True); self.watchdog.start()
            self.fc.begin_takeoff(C.TAKEOFF_ALTITUDE)
            end=time.monotonic()+15
            while True:
                self.last_cycle=time.monotonic(); self._guard(); self.grid.read()
                if abs(self.fc.get_current_location().alt-C.TAKEOFF_ALTITUDE)<C.ALTITUDE_TOLERANCE_M: break
                if time.monotonic()>end: raise FlightError('Takeoff timeout')
                self.fc.pause(0.1)
            self.evidence.write('takeoff_verified',alt_m=self.fc.get_current_location().alt)
            for i,p in enumerate(C.DROP_POINTS):
                self.navigate(*self._point(p,i)); self.align(i+1); self.release(i)
            if not self.target_observed: raise FlightError('Target + 20cm descent not completed; end-zone remains blocked')
            self.final_approach=True
            self.navigate(*self._point(C.LANDING_POINT,4)); self.align(landing=True)
            self.state='LAND'; self.fc.begin_land(); end=time.monotonic()+40; steps=0
            while not self.fc.landed_confirmed():
                self.last_cycle=time.monotonic(); self._guard(); steps+=1
                if time.monotonic()>end or steps>500: raise FlightError('Touchdown and auto-disarm not confirmed')
                # FC owns terminal LAND; no companion velocity/RTL overrides.
                self.fc.pause(0.1)
            self.completed=True; self.airborne=False
            self.evidence.write('mission_complete',landed_and_disarmed=True,release_count=self.completed_drops,competition_score=None)
            return True
        except Exception as exc:
            self.abort(str(exc)); return False
    def close(self):
        self.watchdog_stop.set()
        if self.watchdog and self.watchdog.is_alive() and self.watchdog is not threading.current_thread(): self.watchdog.join(timeout=0.3)
        if self.fc and self.fc.is_armed() and not self.completed and not self.aborted: self.abort('Shutdown before touchdown')
        for resource in (self.vision,self.grid,self.fc):
            if resource:
                try: resource.close()
                except Exception: log.exception('Resource close failed')

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--simulation',action='store_true'); parser.add_argument('--execute',action='store_true'); parser.add_argument('--confirm-real',action='store_true')
    parser.add_argument('--check-config',action='store_true'); parser.add_argument('--port'); parser.add_argument('--baud',type=int)
    args=parser.parse_args()
    if args.simulation and args.execute: parser.error('--simulation and --execute are mutually exclusive')
    if args.execute and not args.confirm_real: parser.error('Real flight requires --execute --confirm-real')
    simulation=args.simulation or not args.execute
    if args.check_config:
        errors=C.validate_config(simulation=False)
        print('\n'.join(errors) if errors else 'Configuration complete; hardware preflight still required')
        return 1 if errors else 0
    if not simulation:
        errors=C.validate_config(simulation=False)
        if errors: log.error('Real flight blocked:\n%s','\n'.join(errors)); return 1
    controller=MissionController(simulation=simulation,port=args.port,baud=args.baud)
    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGINT,controller.request_stop); signal.signal(signal.SIGTERM,controller.request_stop)
    try:
        ok=controller.run(); print('SIMULATION ONLY' if simulation else 'HARDWARE RUN', 'PASS' if ok else 'FAIL',controller.evidence.path)
        return 0 if ok else 1
    finally: controller.close()
if __name__=='__main__':
    logging.basicConfig(level=logging.INFO,format='%(asctime)s %(levelname)s %(message)s')
    raise SystemExit(main())
