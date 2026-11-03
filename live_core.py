"""Short-lived observed occupancy in odom. No prior arena geometry.
Unknown is blocked; ray visibility never establishes global completeness.
"""
import math
import heapq
import numpy as np
from scipy.ndimage import distance_transform_edt, label


def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def clamp(a,lo,hi):return max(lo,min(hi,a))


class Memory:
    def __init__(self,res=.04,radius=.113,margin=.03,extent=3.,ttl=20.):
        self.res,self.radius,self.margin=res,radius,margin
        self.extent,self.ttl=extent,ttl
        self.cells={};self.visits={}
    def key(self,x,y):return math.floor(x/self.res),math.floor(y/self.res)
    def observe(self,origin,rays,now):
        # rays are (world_x, world_y, hit). They must come from valid measurements.
        hits=[]
        for x,y,hit in rays:
            dx,dy=x-origin[0],y-origin[1];d=math.hypot(dx,dy)
            n=max(1,math.ceil(d/(self.res*.4)))
            for i in range(n):
                k=self.key(origin[0]+dx*i/n,origin[1]+dy*i/n)
                self.cells[k]=(0,now)
            if hit:hits.append(self.key(x,y))
            else:self.cells[self.key(x,y)]=(0,now)
        # Endpoint hits win over free-ray updates in this scan.
        for k in hits:self.cells[k]=(100,now)
        self.cells={k:v for k,v in self.cells.items() if now-v[1]<=self.ttl and
                    abs((k[0]+.5)*self.res-origin[0])<self.extent+1 and
                    abs((k[1]+.5)*self.res-origin[1])<self.extent+1}
    def grid(self,pose,now):
        n=math.ceil(2*self.extent/self.res)+1
        centre=self.key(pose[0],pose[1]);offset=(centre[0]-n//2,centre[1]-n//2)
        raw=np.full((n,n),-1,dtype=np.int8)
        for (x,y),(v,t) in self.cells.items():
            i,j=x-offset[0],y-offset[1]
            if 0<=i<n and 0<=j<n and now-t<=self.ttl:raw[j,i]=v
        known=raw==0
        edt=distance_transform_edt(np.pad(known,1,constant_values=False))[1:-1,1:-1]*self.res
        # Cover half diagonal of a blocked cell plus centre-snapping uncertainty.
        free=known & (edt>self.radius+self.margin+math.sqrt(2)*self.res)
        return raw,free,offset
    def start_diagnostic(self,pose,now):
        raw,free,offset=self.grid(pose,now)
        x,y=self.local(pose[:2],offset)
        if not (0<=x<raw.shape[1] and 0<=y<raw.shape[0]):return 'centre outside local grid'
        yy,xx=np.indices(raw.shape)
        d=np.hypot(xx-x,yy-y)*self.res
        def nearest(mask):
            return f'{float(d[mask].min()):.3f}m' if mask.any() else 'none in window'
        cell_names={-1:'UNKNOWN',0:'FREE',100:'OCCUPIED'}
        required=self.radius+self.margin+math.sqrt(2)*self.res
        return (f'centre={cell_names.get(int(raw[y,x]),str(raw[y,x]))}; '
                f'occupied-cell distance={nearest(raw==100)}; '
                f'unknown-cell distance={nearest(raw==-1)}; '
                f'required>{required:.3f}m (radius={self.radius:.3f}, margin={self.margin:.3f}, '
                f'grid allowance={math.sqrt(2)*self.res:.3f}); '
                f'allowed centres={int(free.sum())}')

    def world(self,k,offset):return ((k[0]+offset[0]+.5)*self.res,(k[1]+offset[1]+.5)*self.res)
    def local(self,p,offset):
        k=self.key(*p);return k[0]-offset[0],k[1]-offset[1]


def visible(a,b,free):
    h,w=free.shape
    if not all(0<=p[0]<w and 0<=p[1]<h for p in (a,b)):return False
    steps=max(abs(a[0]-b[0]),abs(a[1]-b[1]))*4+1
    old=a
    for i in range(steps+1):
        p=(round(a[0]+(b[0]-a[0])*i/steps),round(a[1]+(b[1]-a[1])*i/steps))
        if not free[p[1],p[0]]:return False
        if p[0]!=old[0] and p[1]!=old[1] and (not free[p[1],old[0]] or not free[old[1],p[0]]):return False
        old=p
    return True


def route_to(free,start,goal):
    if not visible(start,start,free) or not visible(goal,goal,free):return []
    todo=[(math.dist(start,goal),0.,start)];cost={start:0.};prev={}
    while todo:
        _,d,p=heapq.heappop(todo)
        if d>cost[p]:continue
        if p==goal:
            route=[p]
            while route[-1]!=start:route.append(prev[route[-1]])
            route.reverse();out=[route[0]];i=0
            while i<len(route)-1:
                j=len(route)-1
                while j>i+1 and not visible(route[i],route[j],free):j-=1
                out.append(route[j]);i=j
            return out
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            q=(p[0]+dx,p[1]+dy)
            if not visible(p,q,free):continue
            nd=d+math.hypot(dx,dy)
            if nd<cost.get(q,math.inf):
                cost[q]=nd;prev[q]=p
                heapq.heappush(todo,(nd+math.dist(q,goal),nd,q))
    return []


def select_route(memory,pose,goal,now,failed):
    raw,free,offset=memory.grid(pose,now)
    start=memory.local(pose[:2],offset);end=memory.local(goal,offset)
    if not visible(start,start,free):return [],'START_CLEARANCE',raw,free,offset
    direct=route_to(free,start,end)
    if direct:return [memory.world(q,offset) for q in direct[1:]],'KNOWN_ROUTE',raw,free,offset
    # Find observed reachable space; do not invent a path through occlusions.
    # Label with 4-connectivity: conservative; A* later checks diagonal corners.
    components,_=label(free)
    reachable=components==components[start[1],start[0]]
    candidates=[]
    for y,x in zip(*np.where(reachable)):
        if x%4 or y%4:continue
        q=memory.world((int(x),int(y)),offset)
        if math.dist(pose[:2],q)<.25 or any(math.dist(q,f)<.20 for f in failed):continue
        visits=memory.visits.get(memory.key(*q),0)
        score=math.dist(q,goal)+.05*math.dist(q,pose[:2])+.25*visits
        candidates.append((score,(int(x),int(y))))
    for _,q in sorted(candidates)[:16]:
        route=route_to(free,start,q)
        if route:return [memory.world(k,offset) for k in route[1:]],'OBSERVED_SUBGOAL',raw,free,offset
    return [],'NO_OBSERVED_ROUTE',raw,free,offset


class PendingScans:
    """Bounded timestamp queue; retry TF readiness without blocking callbacks."""
    def __init__(self,limit=8):self.items=[];self.limit=limit
    def add(self,stamp,msg):
        self.items.append((stamp,msg));self.items.sort(key=lambda p:p[0])
        self.items=self.items[-self.limit:]
    def take_ready(self,now,can_transform):
        self.items=[p for p in self.items if -.1<=now-p[0]<=.6]
        for i in range(len(self.items)-1,-1,-1):
            stamp,msg=self.items[i]
            if can_transform(msg):
                self.items=self.items[i+1:]
                return msg
        return None


def swept_safe(memory,pose,v,w,now,horizon=.8):
    """Check circular footprint along commanded arc in observed inflated grid."""
    _,free,offset=memory.grid(pose,now)
    a=memory.local(pose[:2],offset)
    if not visible(a,a,free):return False
    x,y,theta=pose
    for _ in range(16):
        dt=horizon/16
        if abs(w)<1e-8:x+=v*math.cos(theta)*dt;y+=v*math.sin(theta)*dt
        else:
            x+=v/w*(math.sin(theta+w*dt)-math.sin(theta))
            y-=v/w*(math.cos(theta+w*dt)-math.cos(theta))
        theta+=w*dt
        b=memory.local((x,y),offset)
        if not visible(a,b,free):return False
        a=b
    return True


from drive_safety import CommandLease
