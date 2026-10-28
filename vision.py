"""Timestamped asynchronous video input. FPV VTX is not a frame capture device."""
from dataclasses import dataclass
import math, threading, time
from pathlib import Path
import config as C
from serial_comm import FlightError
@dataclass(frozen=True)
class VideoSample:
    sequence: int
    captured_at: float
    frame: object
class CameraStream:
    def __init__(self,port,cv2):
        self.cv2=cv2; self._lock=threading.Lock(); self._stop=threading.Event()
        self.latest=None; self.error=None; self.cap=None
        self.thread=threading.Thread(target=self._run,args=(port,),daemon=True); self.thread.start()
    def _run(self,port):
        try:
            self.cap=self.cv2.VideoCapture(int(port) if str(port).isdigit() else port)
            if not self.cap.isOpened(): raise FlightError('Cannot open camera '+str(port))
            for prop,val in ((self.cv2.CAP_PROP_FRAME_WIDTH,C.CAMERA_WIDTH),(self.cv2.CAP_PROP_FRAME_HEIGHT,C.CAMERA_HEIGHT),(self.cv2.CAP_PROP_FPS,C.CAMERA_FPS),(self.cv2.CAP_PROP_BUFFERSIZE,1)):
                self.cap.set(prop,val)
            seq=0
            while not self._stop.is_set():
                ok,frame=self.cap.read()
                if not ok or frame is None: raise FlightError('Camera frame missing')
                seq+=1
                with self._lock: self.latest=VideoSample(seq,time.monotonic(),frame)
        except Exception as exc: self.error=str(exc)
        finally:
            if self.cap: self.cap.release()
    def read(self):
        with self._lock: sample=self.latest
        if self.error: raise FlightError(self.error)
        if sample is None or time.monotonic()-sample.captured_at>0.4: raise FlightError('Camera missing/stale')
        return sample
    def close(self):
        self._stop.set(); self.thread.join(timeout=0.3)
        # A stalled capture thread cannot hold mission shutdown indefinitely.

class VisionProcessor:
    def __init__(self):
        import cv2, numpy as np
        self.cv2=cv2; self.np=np; self.bottom=None; self.front=None
        try:
            self.collection_templates={i:self._load(C.COLLECTION_TEMPLATE_DIR/f'point_{i}.png') for i in range(1,5)}
            self.target_templates={i:self._load(C.TARGET_TEMPLATE_DIR/f'obstacle_{i}.png') for i in (1,2)}
            self.landing_template=self._load(C.LANDING_TEMPLATE_PATH)
            self.bottom=CameraStream(C.BOTTOM_CAMERA_PORT,cv2); self.front=CameraStream(C.FRONT_CAMERA_PORT,cv2)
        except BaseException: self.close(); raise
    def _load(self,path):
        img=self.cv2.imread(str(path),self.cv2.IMREAD_GRAYSCALE)
        if img is None or img.size<64 or img.std()<8: raise FlightError('Missing/untextured template '+str(path))
        return img
    def read_bottom_frame(self): return self.bottom.read()
    def read_front_frame(self): return self.front.read()
    def _template_match(self,frame,template):
        gray=self.cv2.cvtColor(frame,self.cv2.COLOR_BGR2GRAY); best=None
        # Bounded scale/rotation search; requires validation with real camera dataset.
        for scale in (0.5,0.7,1.,1.4,2.,2.8):
            w,h=round(template.shape[1]*scale),round(template.shape[0]*scale)
            if min(w,h)<8 or w>gray.shape[1] or h>gray.shape[0]: continue
            resized=self.cv2.resize(template,(w,h))
            for angle in (-10,0,10):
                matrix=self.cv2.getRotationMatrix2D((w/2,h/2),angle,1)
                t=self.cv2.warpAffine(resized,matrix,(w,h),borderMode=self.cv2.BORDER_REPLICATE) if angle else resized
                result=self.cv2.matchTemplate(gray,t,self.cv2.TM_CCOEFF_NORMED)
                _,score,_,top=self.cv2.minMaxLoc(result)
                if math.isfinite(score) and score>=C.TARGET_MATCH_THRESHOLD and (best is None or score>best['score']):
                    best={'score':float(score),'center':(top[0]+w/2,top[1]+h/2),'size':(w,h)}
        return best
    def detect_collection_point(self,frame,expected_id):
        label=self._template_match(frame,self.collection_templates[expected_id])
        if not label: return None
        gray=self.cv2.cvtColor(frame,self.cv2.COLOR_BGR2GRAY)
        circles=self.cv2.HoughCircles(self.cv2.GaussianBlur(gray,(5,5),1.2),self.cv2.HOUGH_GRADIENT,1,20,param1=100,param2=30,minRadius=8,maxRadius=min(gray.shape)//2)
        if circles is None: return None
        candidates=[]
        for x,y,r in circles[0]:
            # Identity and ring must belong to the same target; no unrelated largest green blob.
            if math.dist((x,y),label['center'])<=r*0.65:
                candidates.append((math.dist((x,y),label['center'])/r,x,y,r))
        if not candidates: return None
        _,x,y,r=min(candidates)
        return {'point_id':expected_id,'center':(float(x),float(y)),'radius':float(r),'label_score':label['score']}
    def detect_target_image(self,frame):
        matches=[]
        for oid in (1,2):
            match=self._template_match(frame,self.target_templates[oid])
            if match: matches.append({'obstacle_id':oid,**match})
        return max(matches,key=lambda m:m['score']) if matches else None
    def detect_landing_zone(self,frame):
        # Template is central landing marker, not full circle; still visible during descent.
        return self._template_match(frame,self.landing_template)
    def close(self):
        for name in ('bottom','front'):
            stream=getattr(self,name,None)
            if stream: stream.close()

class MockFrame:
    shape=(C.CAMERA_HEIGHT,C.CAMERA_WIDTH,3)
class MockVisionProcessor:
    def __init__(self): self.seq=0; self.dropout=False; self.target=True; self.center_offset=(0,0)
    def _read(self):
        if self.dropout: raise FlightError('Injected camera dropout')
        self.seq+=1; return VideoSample(self.seq,time.monotonic(),MockFrame())
    def read_bottom_frame(self): return self._read()
    def read_front_frame(self): return self._read()
    def center(self): return (C.CAMERA_WIDTH/2+self.center_offset[0],C.CAMERA_HEIGHT/2+self.center_offset[1])
    def detect_collection_point(self,frame,expected_id): return {'point_id':expected_id,'center':self.center(),'radius':30.,'label_score':1.}
    def detect_target_image(self,frame): return {'obstacle_id':1,'center':self.center(),'score':1.} if self.target else None
    def detect_landing_zone(self,frame): return {'center':self.center(),'score':1.}
    def close(self): pass
