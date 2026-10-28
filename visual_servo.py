"""Time-based PID; camera-to-body orientation must be calibrated on bench."""
from dataclasses import dataclass
import time
import config as C
from serial_comm import clamp
@dataclass(frozen=True)
class VisualServoCommand:
    vx: float
    vy: float
    centered: bool
    dx: float
    dy: float
class PixelPID:
    def __init__(self,tolerance_px,*,matrix=None):
        self.tolerance=tolerance_px; self.matrix=matrix if matrix is not None else C.CAMERA_TO_BODY
        if self.matrix is None: raise ValueError('Camera/body matrix is not calibrated')
        self.reset()
    def reset(self): self.integral=[0.,0.]; self.previous=None; self.last_time=None
    def update(self,center,shape,*,now=None):
        now=time.monotonic() if now is None else now
        err=[float(center[0]-shape[1]/2),float(center[1]-shape[0]/2)]
        dt=max(0.01,min(0.25,now-self.last_time)) if self.last_time is not None else 0.1
        deriv=[(err[i]-self.previous[i])/dt if self.previous is not None else 0 for i in (0,1)]
        self.integral=[clamp(self.integral[i]+err[i]*dt,80) for i in (0,1)]
        raw=[C.VISUAL_KP*err[i]+C.VISUAL_KI*self.integral[i]+C.VISUAL_KD*deriv[i] for i in (0,1)]
        v=[sum(self.matrix[i][j]*raw[j] for j in (0,1)) for i in (0,1)]
        length=(v[0]**2+v[1]**2)**0.5; scale=min(1,C.VISUAL_MAX_SPEED_MPS/max(length,1e-9))
        self.previous=err; self.last_time=now
        return VisualServoCommand(v[0]*scale,v[1]*scale,all(abs(x)<=self.tolerance for x in err),*err)
