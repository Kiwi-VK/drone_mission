"""Single MAVLink reader; fresh telemetry, ACKs, cancellable actions. No RTL in cleanup."""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass
import logging, math, threading, time
import config as C
log=logging.getLogger(__name__)
class FlightError(RuntimeError): pass
@dataclass(frozen=True)
class Location:
    north: float
    east: float
    alt: float
    yaw: float = 0.0

def clamp(x,limit): return max(-limit,min(limit,x))

class FlightController:
    def __init__(self,port=None,baud=None):
        from pymavlink import mavutil
        self.mu=mavutil; self.m=mavutil.mavlink
        self.port=port or C.FLIGHT_CONTROLLER_PORT; self.baud=baud or C.FLIGHT_CONTROLLER_BAUD
        self.connection=None; self.connected=False; self._stop=threading.Event()
        self._lock=threading.RLock(); self._tx=threading.Lock(); self._acks=deque(maxlen=100)
        self._seen={}; self._armed=False; self._mode='UNKNOWN'; self._location=None
        self._battery=0.; self._ekf=0; self._landed=None; self._down_range=None
        self._autopilot=None; self._vehicle=None; self._reader_error=None; self._version=None
        self.cancel=lambda:False; self.guard=lambda:None; self._expected_mode=None
        try:
            self.connection=mavutil.mavlink_connection(self.port,baud=self.baud,autoreconnect=False,source_system=250,source_component=191)
            hb=self.connection.wait_heartbeat(timeout=C.FC_HEARTBEAT_TIMEOUT)
            if hb is None: raise FlightError('No autopilot heartbeat: Betaflight MSP is not supported')
            self.target_system=hb.get_srcSystem(); self.target_component=hb.get_srcComponent()
            self._consume(hb); self.connected=True
            self._thread=threading.Thread(target=self._reader_loop,daemon=True); self._thread.start()
            self._sender=threading.Thread(target=self._heartbeat_loop,daemon=True); self._sender.start()
            for mid,hz in ((32,20),(30,20),(193,5),(1,2),(245,5),(132,10)):
                self.command(self.m.MAV_CMD_SET_MESSAGE_INTERVAL,mid,1e6/hz,timeout=2)
        except BaseException:
            self.close(); raise

    def _consume(self,msg):
        if msg.get_srcSystem()!=self.target_system or msg.get_srcComponent()!=self.target_component: return
        kind=msg.get_type(); now=time.monotonic()
        with self._lock:
            self._seen[kind]=now
            if kind=='HEARTBEAT':
                self._armed=bool(msg.base_mode & self.m.MAV_MODE_FLAG_SAFETY_ARMED)
                self._mode=self.mu.mode_string_v10(msg); self._autopilot=msg.autopilot; self._vehicle=msg.type
            elif kind=='LOCAL_POSITION_NED':
                origin=[C.MISSION_DATA.get(k,0) or 0 for k in ('local_origin_north_m','local_origin_east_m','local_origin_down_m')]
                vals=(msg.x-origin[0],msg.y-origin[1],-(msg.z-origin[2]))
                if not all(math.isfinite(v) for v in vals): raise FlightError('Non-finite local position')
                self._location=Location(*vals,self._location.yaw if self._location else 0.)
            elif kind=='ATTITUDE' and self._location:
                if not math.isfinite(msg.yaw): raise FlightError('Non-finite yaw')
                p=self._location; self._location=Location(p.north,p.east,p.alt,msg.yaw)
                self._roll,self._pitch=msg.roll,msg.pitch
            elif kind=='SYS_STATUS':
                self._battery=msg.voltage_battery/1000 if msg.voltage_battery not in (0,65535) else 0.
            elif kind=='EKF_STATUS_REPORT': self._ekf=msg.flags
            elif kind=='EXTENDED_SYS_STATE': self._landed=msg.landed_state
            elif kind=='DISTANCE_SENSOR' and msg.orientation==25:
                valid=msg.min_distance<msg.current_distance<msg.max_distance and getattr(msg,'signal_quality',0)!=1
                self._down_range=msg.current_distance/100 if valid else None
                self._seen['DOWN_RANGE']=now
            elif kind=='COMMAND_ACK': self._acks.append((now,int(msg.command),int(msg.result)))
            elif kind=='AUTOPILOT_VERSION': self._version=msg.flight_sw_version

    def _reader_loop(self):
        while not self._stop.is_set():
            try:
                msg=self.connection.recv_match(blocking=True,timeout=0.2)
                if msg is not None: self._consume(msg)
            except Exception as exc:
                self._reader_error=str(exc); log.exception('MAVLink receive failed'); return

    def _heartbeat_loop(self):
        while not self._stop.is_set():
            try:
                with self._tx:
                    self.connection.mav.heartbeat_send(self.m.MAV_TYPE_GCS,self.m.MAV_AUTOPILOT_INVALID,0,0,self.m.MAV_STATE_ACTIVE)
            except Exception as exc: self._reader_error=str(exc); return
            self._stop.wait(0.5)

    def age(self,key): return time.monotonic()-self._seen.get(key,-1e9)
    def health_errors(self,*,airborne=False):
        errors=[]
        if self._reader_error: errors.append('MAVLink reader failed: '+self._reader_error)
        if self._autopilot!=self.m.MAV_AUTOPILOT_ARDUPILOTMEGA: errors.append('Requires ArduCopter, not Betaflight/PX4')
        if self._vehicle not in (2,3,4,13,14,15,29,35): errors.append('Requires multicopter firmware')
        for key,limit in (('HEARTBEAT',C.HEARTBEAT_MAX_AGE_S),('LOCAL_POSITION_NED',C.TELEMETRY_MAX_AGE_S),('ATTITUDE',C.TELEMETRY_MAX_AGE_S),('SYS_STATUS',2.),('EKF_STATUS_REPORT',1.),('EXTENDED_SYS_STATE',1.),('DOWN_RANGE',0.5)):
            if self.age(key)>limit: errors.append('Missing/stale telemetry: '+key)
        # EKF attitude + velocity + horizontal position + vertical position; no GPS-only shortcut.
        if not (self._ekf&1 and self._ekf&2 and self._ekf&4 and self._ekf&(8|16) and self._ekf&(32|64)) or self._ekf&128:
            errors.append('EKF indoor position/velocity/height invalid or constant-position mode')
        if self._battery<=0 or self._battery<C.BATTERY_LOW_VOLTAGE: errors.append('Battery missing/low')
        if self._down_range is None: errors.append('Downward range invalid (MT-06 protocol/mounting not commissioned)')
        if abs(getattr(self,'_roll',0))>0.20 or abs(getattr(self,'_pitch',0))>0.20: errors.append('Attitude outside calibrated vision/range envelope')
        if airborne and (not self._armed or self._mode!=self._expected_mode): errors.append('Pilot/FC mode or arm state changed; companion yields control')
        return errors

    def checkpoint(self):
        if self.cancel(): raise FlightError('Stop requested')
        self.guard()

    def pause(self,seconds,*,check=True):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if check: self.checkpoint()
            time.sleep(min(0.02,max(0,end-time.monotonic())))

    def command(self,cmd,*params,timeout=2.,check=True):
        if check: self.checkpoint()
        start=time.monotonic(); values=list(params)+[0.]*(7-len(params))
        with self._tx: self.connection.mav.command_long_send(self.target_system,self.target_component,cmd,0,*values)
        while time.monotonic()-start<timeout:
            if check: self.checkpoint()
            with self._lock: replies=[r for t,c,r in self._acks if c==cmd and t>=start]
            if replies:
                if replies[-1]==self.m.MAV_RESULT_ACCEPTED: return True
                if replies[-1]!=self.m.MAV_RESULT_IN_PROGRESS: raise FlightError(f'Command {cmd} rejected: {replies[-1]}')
            self.pause(0.02,check=check)
        raise FlightError(f'Command ACK timeout: {cmd}')

    def set_mode(self,name,*,emergency=False):
        mapping=self.connection.mode_mapping() or {}
        if name not in mapping: raise FlightError('Unsupported mode '+name)
        with self._tx: self.connection.set_mode(mapping[name])
        end=time.monotonic()+(1.5 if emergency else 4.)
        while time.monotonic()<end:
            if self._mode==name and self.age('HEARTBEAT')<C.HEARTBEAT_MAX_AGE_S:
                self._expected_mode=name; return True
            self.pause(0.04,check=not emergency)
        raise FlightError('Mode did not become '+name)

    def arm(self):
        self.set_mode('GUIDED')
        self.command(self.m.MAV_CMD_COMPONENT_ARM_DISARM,1,0)
        end=time.monotonic()+8
        while time.monotonic()<end:
            if self._armed: return True
            self.pause(0.05)
        raise FlightError('Arm state not confirmed')

    def begin_takeoff(self,alt):
        return self.command(self.m.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,alt)

    def send_velocity(self,north,east,down=0):
        # All mission movement uses local NED velocity. One arbiter chooses it per cycle.
        vals=(north,east,down)
        if not all(math.isfinite(v) for v in vals): raise FlightError('Invalid velocity')
        if math.hypot(north,east)>C.MAX_FORWARD_SPEED_MPS+1e-6 or abs(down)>C.MAX_VERTICAL_SPEED_MPS+1e-6: raise FlightError('Velocity exceeds limits')
        mask=1|2|4|64|128|256|1024|2048
        with self._tx:
            self.connection.mav.set_position_target_local_ned_send(int(time.monotonic()*1000)&0xffffffff,self.target_system,self.target_component,self.m.MAV_FRAME_LOCAL_NED,mask,0,0,0,north,east,down,0,0,0,0,0)

    def hold_position(self): self.send_velocity(0,0,0)
    def set_servo(self,channel,pwm): return self.command(self.m.MAV_CMD_DO_SET_SERVO,channel,pwm)
    def begin_land(self): return self.set_mode('LAND')
    def landed_confirmed(self):
        return (self.age('HEARTBEAT')<C.HEARTBEAT_MAX_AGE_S and self.age('EXTENDED_SYS_STATE')<1. and not self._armed and self._landed==self.m.MAV_LANDED_STATE_ON_GROUND)
    def emergency_land(self):
        # Never override a pilot/FC switch, never RTL into a 2m roof.
        if self._armed and self._mode in ('GUIDED','LAND') and self.age('HEARTBEAT')<C.HEARTBEAT_MAX_AGE_S:
            self.set_mode('LAND',emergency=True); return True
        return False
    def get_current_location(self):
        with self._lock: return self._location
    def get_down_range(self): return self._down_range
    def is_armed(self): return self._armed
    def close(self):
        self._stop.set()
        for name in ('_thread','_sender'):
            t=getattr(self,name,None)
            if t and t.is_alive(): t.join(timeout=0.7)
        if self.connection: self.connection.close()
        self.connected=False

class MockFlightController:
    """Deterministic kinematic model; explicitly not SITL or flight evidence."""
    def __init__(self):
        self.connected=True; self._armed=False; self._mode='STABILIZE'; self._location=Location(0,0,0)
        self._expected_mode='GUIDED'; self.cancel=lambda:False; self.guard=lambda:None
        self.velocity_log=[]; self.mode_log=[]; self.servo_log=[]; self._landing=False
        self.fault=None; self.frozen=False; self.pause_count=0
    def health_errors(self,*,airborne=False):
        if self.fault: return [self.fault]
        if airborne and (not self._armed or self._mode!=self._expected_mode): return ['Pilot/FC mode or arm state changed; companion yields control']
        return []
    def checkpoint(self):
        if self.cancel(): raise FlightError('Stop requested')
        self.guard()
    def pause(self,seconds,*,check=True):
        if check: self.checkpoint()
        self.pause_count+=1
        if self._landing:
            p=self._location; alt=max(0,p.alt-0.1)
            self._location=Location(p.north,p.east,alt,p.yaw)
            if alt==0: self._armed=False
    def set_mode(self,name,*,emergency=False):
        self.mode_log.append(name); self._mode=name; self._expected_mode=name
        if name=='LAND': self._landing=True
        return True
    def arm(self): self.set_mode('GUIDED'); self._armed=True; return True
    def begin_takeoff(self,alt):
        p=self._location; self._location=Location(p.north,p.east,alt,p.yaw); return True
    def send_velocity(self,north,east,down=0):
        self.velocity_log.append((north,east,down)); p=self._location
        if not self.frozen: self._location=Location(p.north+north*0.1,p.east+east*0.1,max(0,p.alt-down*0.1),p.yaw)
    def hold_position(self): self.send_velocity(0,0,0)
    def set_servo(self,ch,pwm): self.servo_log.append((ch,pwm)); return True
    def begin_land(self): return self.set_mode('LAND')
    def landed_confirmed(self): return not self._armed and self._location.alt==0 and self._mode=='LAND'
    def emergency_land(self):
        if self._armed and self._mode in ('GUIDED','LAND'): self.set_mode('LAND'); return True
        return False
    def get_current_location(self): return self._location
    def get_down_range(self): return max(0.05,self._location.alt)
    def is_armed(self): return self._armed
    def close(self): self.connected=False

def get_flight_controller(*,simulation=False,port=None,baud=None):
    return MockFlightController() if simulation else FlightController(port,baud)
