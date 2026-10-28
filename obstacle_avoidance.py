"""Live occupancy grid + footprint-inflated A*. Unknown cells are blocked.
Grid is produced from current sensor observations; never load A/B bypass routes.
"""
from __future__ import annotations
from dataclasses import dataclass
import heapq, json, math, socket, time
import config as C
from serial_comm import FlightError

@dataclass(frozen=True)
class OccupancyGrid:
    sequence: int
    received_at: float
    resolution: float
    origin: tuple
    width: int
    height: int
    cells: tuple  # 0=observed free, 1=occupied, -1=unknown

    @classmethod
    def parse(cls,data,received_at=None):
        d=json.loads(data)
        w,h=d['width'],d['height']; res=d['resolution_m']; origin=d['origin_ne']
        if type(w)!=int or type(h)!=int or w<3 or h<3 or w*h>C.GRID_MAX_CELLS: raise ValueError('Invalid grid size')
        if not C.number(res) or not 0.05<=res<=0.25: raise ValueError('Invalid resolution')
        if len(origin)!=2 or not all(C.number(x) for x in origin): raise ValueError('Invalid origin')
        cells=d['cells']
        if len(cells)!=w*h or any(type(v)!=int or v not in (-1,0,1) for v in cells): raise ValueError('Invalid cells')
        if d.get('frame')!='takeoff_NED' or d.get('coverage')!='full_airframe_height': raise ValueError('Grid frame or vertical coverage not commissioned')
        # Producer clock uses Linux CLOCK_MONOTONIC on this same Pi; delayed packets rejected.
        now=time.monotonic() if received_at is None else received_at
        if not C.number(d.get('sample_monotonic_s')) or not 0<=now-d['sample_monotonic_s']<=C.GRID_MAX_AGE_S: raise ValueError('Stale/foreign-clock grid')
        if type(d.get('sequence'))!=int or d['sequence']<0: raise ValueError('Invalid sequence')
        return cls(d['sequence'],d['sample_monotonic_s'],res,tuple(origin),w,h,tuple(cells))

    def index(self,n,e): return (int(math.floor((e-self.origin[1])/self.resolution)),int(math.floor((n-self.origin[0])/self.resolution)))
    def point(self,cell):
        x,y=cell; return (self.origin[0]+(y+0.5)*self.resolution,self.origin[1]+(x+0.5)*self.resolution)
    def value(self,x,y): return self.cells[y*self.width+x] if 0<=x<self.width and 0<=y<self.height else -1

class GridReceiver:
    def __init__(self):
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1',C.GRID_BIND_PORT)); self.sock.setblocking(False)
        self.latest=None; self.last_error='No range grid received'
    def read(self):
        # Bounded drain: malicious/faulty producer cannot monopolize mission control.
        for _ in range(8):
            try: packet,_=self.sock.recvfrom(65507)
            except BlockingIOError: break
            try:
                grid=OccupancyGrid.parse(packet)
                if self.latest and grid.sequence<=self.latest.sequence: raise ValueError('Replayed/out-of-order grid')
                self.latest=grid; self.last_error=None
            except (ValueError,KeyError,TypeError) as exc: self.last_error=str(exc)
        if self.last_error: raise FlightError('Range grid invalid: '+self.last_error)
        if self.latest is None or time.monotonic()-self.latest.received_at>C.GRID_MAX_AGE_S: raise FlightError('Range grid missing/stale; not free space')
        return self.latest
    def close(self): self.sock.close()

class LocalObstaclePlanner:
    def __init__(self,*,simulation=False): self.simulation=simulation; self.state='NAVIGATE'; self.last_path=[]
    def blocked(self,g,cell,allow_end=False):
        x,y=cell; n,e=g.point(cell)
        if not self.simulation and not C.inside_field(n,e): return True
        # Inflate occupancy, unknown and raster uncertainty by full prop radius + clearance.
        radius=C.FRAME_SPAN_M/2+C.OBSTACLE_CLEARANCE_M+g.resolution*0.71
        steps=math.ceil(radius/g.resolution)
        for dx in range(-steps,steps+1):
            for dy in range(-steps,steps+1):
                if math.hypot(dx,dy)*g.resolution<=radius and g.value(x+dx,y+dy)!=0: return True
        end=C.FIELD.get('end_zone')
        if not allow_end and end and all(C.number(v) for v in end):
            if end[0]-radius<=n<=end[1]+radius and end[2]-radius<=e<=end[3]+radius: return True
        return False
    def safe_segment(self,g,a,b,allow_end=False):
        distance=math.dist(a,b); count=max(1,math.ceil(distance/(g.resolution/3)))
        return all(not self.blocked(g,g.index(a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count),allow_end) for i in range(count+1))
    def plan(self,g,start,goal,*,allow_end=False):
        if time.monotonic()-g.received_at>C.GRID_MAX_AGE_S: raise FlightError('Stale grid')
        s,t=g.index(*start),g.index(*goal)
        cache={}
        def blocked(c):
            if c not in cache: cache[c]=self.blocked(g,c,allow_end)
            return cache[c]
        if blocked(s): raise FlightError('Current footprint intersects obstacle/unknown/boundary: brake')
        if blocked(t): raise FlightError('Destination not observed free: brake; survey/range coverage required')
        if self.safe_segment(g,start,goal,allow_end): self.state='NAVIGATE'; self.last_path=[goal]; return self.last_path
        self.state='AVOID'; queue=[(0,s)]; costs={s:0}; parent={}
        while queue:
            _,cur=heapq.heappop(queue)
            if cur==t: break
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt=(cur[0]+dx,cur[1]+dy)
                if blocked(nxt): continue
                if dx and dy and (blocked((cur[0]+dx,cur[1])) or blocked((cur[0],cur[1]+dy))): continue
                score=costs[cur]+math.hypot(dx,dy)
                if score<costs.get(nxt,math.inf):
                    costs[nxt]=score; parent[nxt]=cur
                    heapq.heappush(queue,(score+math.dist(nxt,t),nxt))
        if t not in costs: raise FlightError('No observed collision-free path; stop instead of guessing A/B')
        path=[t]
        while path[-1]!=s: path.append(parent[path[-1]])
        self.last_path=[g.point(c) for c in reversed(path[:-1])]
        return self.last_path
    def arbitrate(self,g,start,desired,*,allow_end=False):
        # Every velocity (navigation, alignment, descent, release hold) passes here.
        n,e=desired; predicted=(start[0]+n*0.8,start[1]+e*0.8)
        if not self.safe_segment(g,start,predicted,allow_end):
            self.state='BRAKE'; raise FlightError('Desired velocity enters blocked/unknown footprint')
        return desired

class MockGridReceiver:
    def __init__(self): self.sequence=0; self.obstacles=[(1.0,0.0,0.12)]; self.stale=False
    def read(self):
        self.sequence+=1; w=h=60; res=0.1; origin=(-3.,-3.); cells=[]
        for y in range(h):
            for x in range(w):
                n,e=origin[0]+(y+0.5)*res,origin[1]+(x+0.5)*res
                cells.append(int(any(math.hypot(n-a,e-b)<r for a,b,r in self.obstacles)))
        grid=OccupancyGrid(self.sequence,time.monotonic()-(2 if self.stale else 0),res,origin,w,h,tuple(cells))
        if self.stale: raise FlightError('Range grid missing/stale; not free space')
        return grid
    def close(self): pass
