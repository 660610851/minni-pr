"""Experimental map-free APF + live-scan temporary goals.
No unknown-grid gate; no claim of completeness or U-trap success.
Footprint assumes base_footprint is at centre of a 138 x 178 mm rectangle.
"""
import math
import numpy as np
from collections import deque
from live_core import wrap,clamp
from drive_safety import MAX_LINEAR, MAX_ANGULAR


def body_points(pose,points):
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return [(c*(x-pose[0])+s*(y-pose[1]),-s*(x-pose[0])+c*(y-pose[1])) for x,y in points]


def footprint_gap(x,y,half_length=.069,half_width=.089):
    # Signed distance from point to axis-aligned physical rectangle.
    dx,dy=abs(x)-half_length,abs(y)-half_width
    return math.hypot(max(dx,0.),max(dy,0.))+min(max(dx,dy),0.)


def swept_clear(points,v,w,margin=.03,horizon=.8):
    # Re-evaluate rectangle orientation at every predicted pose; no grid inflation.
    for i in range(17):
        t=horizon*i/16
        if abs(w)<1e-8:x,y=v*t,0.
        else:x,y=v/w*math.sin(w*t),v/w*(1-math.cos(w*t))
        c,s=math.cos(w*t),math.sin(w*t)
        for px,py in points:
            dx,dy=px-x,py-y
            if footprint_gap(c*dx+s*dy,-s*dx+c*dy)<=margin:return False
    return True


def corridor_clear(points,target,pad):
    length=math.hypot(*target)
    if length<1e-9:return all(math.hypot(x,y)>pad for x,y in points)
    ux,uy=target[0]/length,target[1]/length
    return all(math.hypot(x-clamp(x*ux+y*uy,0.,length)*ux,
                          y-clamp(x*ux+y*uy,0.,length)*uy)>pad for x,y in points)


def is_stalled(history,pose,active):
    if len(history)<3 or history[-1][0]-history[0][0]<=7.5:return False
    gd=math.dist(pose[:2],active)
    if history[0][2]-gd>=.03:return False
    if max(math.dist(pose[:2],p) for _,p,_,_ in history)>=.15:return False
    first=history[0]
    old_error=abs(wrap(math.atan2(active[1]-first[1][1],active[0]-first[1][0])-first[3]))
    new_error=abs(wrap(math.atan2(active[1]-pose[1],active[0]-pose[0])-pose[2]))
    signs=[]
    for a,b in zip(history,list(history)[1:]):
        delta=wrap(b[3]-a[3])
        if abs(delta)>.01:
            sign=1 if delta>0 else -1
            if not signs or signs[-1]!=sign:signs.append(sign)
    oscillating=len(signs)>=3
    # A productive initial turn may require >8 s; do not misclassify it.
    aligning=old_error-new_error>.15 and not oscillating
    return not aligning


class Reactive:
    def __init__(self,max_v=MAX_LINEAR,max_w=MAX_ANGULAR,margin=.03,goal_obstacle_clearance=.02):
        self.max_v,self.max_w,self.margin=clamp(max_v,0.,MAX_LINEAR),clamp(max_w,0.,MAX_ANGULAR),margin
        self.goal_obstacle_clearance=goal_obstacle_clearance
        self.goal=None;self.temp=None;self.state='IDLE';self.history=deque();self.aligning=False;self.path_memory=deque(maxlen=256)
        self.failed=[];self.visited=[];self.attempts=0;self.last_w=0.;self.recent_obstacles={}
    def set_goal(self,goal,now):
        self.goal=goal;self.temp=None;self.started=now;self.history.clear()
        self.failed=[];self.visited=[];self.attempts=0;self.state='APF';self.last_w=0.;self.aligning=False;self.path_memory.clear()
    def stop(self,state):
        self.goal=None;self.temp=None;self.state=state;self.last_w=0.
        return 0.,0.
    def select_temp(self,pose,points,now):
        self.attempts+=1
        candidates=[]
        samples=np.asarray(points,dtype=float).reshape(-1,2)
        for k in range(36):
            a=k*2*math.pi/36
            # Require a corridor clear of measured endpoints, allow retreat.
            for radius in (.25,.4,.55):
                ux,uy=math.cos(a),math.sin(a)
                # Conservative swept circular envelope for a straight subgoal probe.
                pad=math.hypot(.069,.089)+self.margin
                # Same endpoint/capsule test, vectorised to avoid long Python
                # loops in the sensor/control executor on Raspberry Pi.
                projection=np.clip(samples[:,0]*ux+samples[:,1]*uy,0.,radius)
                distances=np.hypot(samples[:,0]-projection*ux,samples[:,1]-projection*uy)
                clearance=min(.6,float(distances.min())) if len(samples) else .6
                if len(samples) and np.any(distances<=pad):continue
                theta=pose[2]+a;q=(pose[0]+radius*math.cos(theta),pose[1]+radius*math.sin(theta))
                if any(math.dist(q,f)<.18 for f in self.failed):continue
                visits=sum(math.dist(q,p)<.2 for p in self.visited)
                if visits>=1:continue
                # Discourage endpoints on previously travelled portions, not
                # just reached temporary targets. Keep blocked corridors rejected.
                repeats=sum(max(0.,1.-math.dist(q,p)/.25) for p in self.path_memory)
                score=1.5*min(clearance,.4)+.3*radius-.18*math.dist(q,self.goal)-.15*visits-.3*repeats
                candidates.append((score,q))
        if not candidates:return False
        self.temp=max(candidates,key=lambda item:item[0])[1]
        turn=abs(wrap(math.atan2(self.temp[1]-pose[1],self.temp[0]-pose[0])-pose[2]))
        self.temp_budget=min(45.,max(15.,turn/max(.01,self.max_w)+math.dist(pose[:2],self.temp)/max(.01,self.max_v)+5.))
        self.temp_started=now;self.history.clear();self.state='TEMP_GOAL';return True
    def heading_velocity(self,error,distance):
        # Finish alignment before translating. Separate entry/exit angles prevent
        # repeated drive/turn switching around one threshold.
        if abs(error)>.30:self.aligning=True
        elif abs(error)<.10:self.aligning=False
        if self.aligning:return 0.
        return self.max_v*max(0.,math.cos(error))*min(1.,distance/.3)
    def step(self,pose,world_points,now,data_age=0.):
        if self.goal is None:return 0.,0.
        if not self.path_memory or math.dist(pose[:2],self.path_memory[-1])>=.12:
            self.path_memory.append(tuple(pose[:2]))
        # Preserve recently seen endpoints when a thin/close panel loses returns.
        # World coordinates are local odometry; this is NOT a persistent map.
        self.recent_obstacles={k:value for k,value in self.recent_obstacles.items() if now-value[0]<=2.}
        for x,y in world_points:
            self.recent_obstacles[(round(x/.02),round(y/.02))]=(now,x,y)
        points=body_points(pose,[(x,y) for _,x,y in self.recent_obstacles.values()])
        if not world_points:return self.stop('NO_VALID_OBSTACLE_RETURNS')
        if min(footprint_gap(*p) for p in points)<=self.margin:return self.stop('FOOTPRINT_CLEARANCE_STOP')
        # Point-goal rejection is separate from physical footprint clearance.
        # Accepting a point goal does not certify that the robot can occupy it.
        if min(math.dist(self.goal,p) for p in world_points)<=self.goal_obstacle_clearance:
            return self.stop('GOAL_OCCUPIED_BY_OBSERVATION')
        if math.dist(pose[:2],self.goal)<=.10:return self.stop('REACHED')
        if self.temp is not None:
            if math.dist(pose[:2],self.temp)<.10:
                self.visited.append(self.temp);self.temp=None;self.history.clear()
                target=body_points(pose,[self.goal])[0]
                if corridor_clear(points,target,math.hypot(.069,.089)+self.margin):
                    self.state='APF'
                elif not self.select_temp(pose,points,now):
                    return self.stop('NO_TEMPORARY_TARGET')
            elif now-self.temp_started>self.temp_budget:
                self.failed.append(self.temp);self.temp=None
                if not self.select_temp(pose,points,now):return self.stop('NO_TEMPORARY_TARGET')
        active=self.temp or self.goal;gd=math.dist(pose[:2],active)
        self.history.append((now,pose[:2],gd,pose[2]))
        while self.history and now-self.history[0][0]>8:self.history.popleft()
        stalled=is_stalled(self.history,pose,active)
        if stalled:
            if self.temp:self.failed.append(self.temp)
            if not self.select_temp(pose,points,now):return self.stop('RECOVERY_FAILED')
            active=self.temp;gd=math.dist(pose[:2],active)
        dx,dy=active[0]-pose[0],active[1]-pose[1];c,s=math.cos(pose[2]),math.sin(pose[2])
        fx,fy=c*dx+s*dy,-s*dx+c*dy
        # One nearest return per angular sector reduces angular-density bias.
        sectors={}
        for x,y in points:
            a=math.atan2(y,x);k=int((a+math.pi)/(2*math.pi)*36)%36
            if k not in sectors or math.hypot(x,y)<math.hypot(*sectors[k]):sectors[k]=(x,y)
        for x,y in sectors.values():
            gap=max(.015,footprint_gap(x,y));d=math.hypot(x,y)
            if gap<.35 and d>0:
                gain=min(5.,.015*(1/gap-1/.35)/gap**2)*min(1.,(gd/.45)**2)
                fx-=gain*x/d/6;fy-=gain*y/d/6
        error=math.atan2(fy,fx)
        self.heading_error=error;self.hold_reason='NONE'
        w=clamp(1.5*error,-self.max_w,self.max_w)
        w=clamp(w,self.last_w-.16,self.last_w+.16)
        v=self.heading_velocity(error,gd)
        if self.aligning:self.hold_reason='ALIGNING'
        # Main goal speed is not a floor. Slow BEFORE the stop margin, based
        # on observed forward footprint corridor and data age.
        front=[x-.069 for x,y in points if x>=.069 and abs(y)<=.089+self.margin]
        if front:
            v=min(v,max(0.,(min(front)-self.margin)/(1.2+max(0.,data_age))))
        if self.temp is not None:v=min(v,self.max_v)
        horizon=.8+max(0.,data_age)
        if not swept_clear(points,v,w,self.margin,horizon):
            v=0.;self.hold_reason='PREDICTED_PATH_BLOCKED'
            if not swept_clear(points,0.,w,self.margin,horizon):return self.stop('ROTATION_CLEARANCE_STOP')
            # Do not wait until touching the obstacle to choose an escape target.
            if self.temp is None:
                if self.select_temp(pose,points,now):self.state='TEMP_GOAL'
                else:return self.stop('NO_TEMPORARY_TARGET')
        self.last_w=w
        return v,w
