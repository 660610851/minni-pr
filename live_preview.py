#!/usr/bin/env python3
"""Observed-space APF preview: no prior map, no /cmd_vel publisher."""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_ros import Buffer,TransformListener,TransformException
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped,TwistStamped
from nav_msgs.msg import OccupancyGrid,Path
from visualization_msgs.msg import Marker,MarkerArray
from live_core import Memory,select_route,visible,wrap,clamp,PendingScans


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class Live(Node):
    def __init__(self,node_name='apf_live_preview'):
        super().__init__(node_name)
        self.declare_parameter('robot_radius',.113)
        self.declare_parameter('margin',.03)
        self.memory=Memory(radius=self.get_parameter('robot_radius').value,margin=self.get_parameter('margin').value)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.goal=None;self.route=[];self.failed=[];self.pose=None;self.points=[]
        self.last_scan=0.;self.last_processed=0.;self.last_log=0.;self.reason='WAIT_SCAN'
        self.pending=PendingScans()
        self.raw_received=0.;self.raw_stamp=None;self.raw_count=0;self.accepted_count=0;self.sensor_error='WAIT_FIRST_SCAN'
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        for topic in ('/goal_pose','/move_base_simple/goal'):
            self.create_subscription(PoseStamped,topic,self.new_goal,10)
        self.create_subscription(PoseStamped,'/apf/reset_memory',self.reset_memory,10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub=self.create_publisher(OccupancyGrid,'/apf/live_grid',qos)
        self.allowed_pub=self.create_publisher(OccupancyGrid,'/apf/live_allowed',qos)
        self.path_pub=self.create_publisher(Path,'/apf/live_path',qos)
        self.mark_pub=self.create_publisher(MarkerArray,'/apf/live_goals',qos)
        self.cmd_pub=self.create_publisher(TwistStamped,'/apf/live_cmd_preview',10)
        self.create_timer(.2,self.tick)
        self.create_timer(.03,self.process_pending)
        if node_name=='apf_live_preview':
            self.get_logger().info('PREVIEW ONLY. No /cmd_vel publisher. Frame=odom. Reads live /scan + TF; no /map subscription.')

    def reset_memory(self,msg):
        self.memory.cells.clear();self.memory.visits.clear();self.goal=None;self.route=[];self.failed=[]
        self.pending.items.clear();self.pose=None;self.points=[];self.last_scan=0.
        self.reason='MEMORY_RESET';self.clear_markers()

    def new_goal(self,msg):
        if msg.header.frame_id!='odom':
            self.get_logger().warn('Goal rejected: set RViz Fixed Frame to odom');return
        p=(msg.pose.position.x,msg.pose.position.y)
        if not all(math.isfinite(v) for v in p):return
        self.goal=p;self.route=[];self.failed=[];self.reason='GOAL_ACCEPTED'
        suffix='; robot remains stationary in preview' if self.get_name()=='apf_live_preview' else '; control goal received'
        self.get_logger().info(f'Goal accepted {p}'+suffix)

    def scan(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        self.raw_received=time.monotonic();self.raw_stamp=stamp;self.raw_count+=1
        self.pending.add(stamp,msg)

    def process_pending(self):
        now=time.monotonic()
        if now-self.last_processed<.12:return
        ros_now=self.get_clock().now().nanoseconds/1e9
        def ready(msg):
            stamp=Time.from_msg(msg.header.stamp)
            return (self.tf.can_transform('odom',msg.header.frame_id,stamp) and
                    self.tf.can_transform('odom','base_footprint',stamp))
        msg=self.pending.take_ready(ros_now,ready)
        if msg is None:
            if now-self.last_scan>.6:
                self.reason='WAIT_TF_OR_SCAN: waiting for timestamp-matched TF'
                self.sensor_error='NO_TIMESTAMP_MATCHED_TF_OR_FRESH_SCAN'
                if self.pending.items:
                    sample=self.pending.items[-1][1]
                    try:
                        stamp=Time.from_msg(sample.header.stamp)
                        self.tf.lookup_transform('odom',sample.header.frame_id,stamp)
                        self.tf.lookup_transform('odom','base_footprint',stamp)
                    except TransformException as e:self.sensor_error=str(e)
            return
        self.process_scan(msg)

    def process_scan(self,msg):
        now=time.monotonic()
        try:
            stamp=Time.from_msg(msg.header.stamp)
            age=(self.get_clock().now()-stamp).nanoseconds/1e9
            if not -.1<=age<=.6:raise ValueError(f'SCAN_AGE={age:.3f}s')
            tr=self.tf.lookup_transform('odom',msg.header.frame_id,stamp)
            bt=self.tf.lookup_transform('odom','base_footprint',stamp)
            for q in (tr.transform.rotation,bt.transform.rotation):
                if abs(q.x)>.05 or abs(q.y)>.05:raise ValueError('NONPLANAR_TF')
            sy=yaw(tr.transform.rotation);t=tr.transform.translation;b=bt.transform.translation
            rays=[];points=[];coverage=set();valid=0
            for i,r in enumerate(msg.ranges):
                a=msg.angle_min+i*msg.angle_increment+sy
                if math.isfinite(r) and max(.03,msg.range_min)<=r<=msg.range_max:
                    hit=r<=2.8;rr=min(r,2.8)
                elif math.isinf(r) and r>0:
                    hit=False;rr=min(msg.range_max,2.8)
                    if rr<=.03:continue
                else:continue
                valid+=1;coverage.add(int((a%(2*math.pi))/(math.pi/4)))
                x,y=t.x+rr*math.cos(a),t.y+rr*math.sin(a)
                rays.append((x,y,hit))
                if hit:points.append((x,y))
            if valid<60 or len(coverage)<8:raise ValueError('INSUFFICIENT_SCAN_COVERAGE')
            if getattr(self,'accumulate_memory',True):
                self.memory.observe((t.x,t.y),rays,now)
            self.pose=(b.x,b.y,yaw(bt.transform.rotation));self.points=points
            self.last_scan=now-max(0.,age)
            self.last_processed=now
            self.accepted_count+=1;self.sensor_error='OK'
            if self.reason.startswith(('WAIT_','INSUFFICIENT')):self.reason='SCAN_READY'
        except (TransformException,ValueError) as e:
            self.reason='WAIT_TF_OR_SCAN: '+str(e)
            self.sensor_error=str(e)

    def sensor_diagnostic(self):
        now=time.monotonic()
        received='never' if self.raw_received==0 else f'{now-self.raw_received:.3f}s'
        accepted='never' if self.last_scan==0 else f'{now-self.last_scan:.3f}s'
        source='never' if self.raw_stamp is None else f'{self.get_clock().now().nanoseconds/1e9-self.raw_stamp:.3f}s'
        return (f'scan_received_age={received}; accepted_age={accepted}; source_age={source}; '
                f'counts={self.raw_count}/{self.accepted_count}; queued={len(self.pending.items)}; sensor={self.sensor_error}')

    def clear_markers(self):
        m=Marker();m.action=Marker.DELETEALL
        self.mark_pub.publish(MarkerArray(markers=[m]))
        path=Path();path.header.frame_id='odom';path.header.stamp=self.get_clock().now().to_msg()
        self.path_pub.publish(path)

    def tick(self):
        now=time.monotonic();v=w=0.
        if self.pose is None or now-self.last_scan>.6:
            self.route=[];self.clear_markers()
            if not self.reason.startswith('WAIT_TF_OR_SCAN'):self.reason='WAIT_FRESH_SCAN_AND_TF'
        else:
            raw,free,offset=self.memory.grid(self.pose,now)
            grid=OccupancyGrid();grid.header.frame_id='odom';grid.header.stamp=self.get_clock().now().to_msg()
            grid.info.resolution=self.memory.res;grid.info.height,grid.info.width=raw.shape
            grid.info.origin.position.x=offset[0]*self.memory.res;grid.info.origin.position.y=offset[1]*self.memory.res
            grid.info.origin.orientation.w=1.;grid.data=raw.ravel().tolist();self.grid_pub.publish(grid)
            allowed=OccupancyGrid();allowed.header=grid.header;allowed.info=grid.info
            allowed.data=np.where(free,0,100).astype(np.int8).ravel().tolist();self.allowed_pub.publish(allowed)
            if self.goal is not None:
                if math.dist(self.pose[:2],self.goal)<.10:
                    self.goal=None;self.route=[];self.reason='REACHED';self.clear_markers()
                else:
                    while self.route and math.dist(self.pose[:2],self.route[0])<.10:self.route.pop(0)
                    if self.route:
                        a=self.memory.local(self.pose[:2],offset);b=self.memory.local(self.route[0],offset)
                        if not visible(a,b,free):self.route=[]
                    if not self.route:
                        self.route,self.reason,raw,free,offset=select_route(self.memory,self.pose,self.goal,now,self.failed)
                    if self.route:
                        target=self.route[0];gx,gy=target[0]-self.pose[0],target[1]-self.pose[1]
                        fx,fy=gx,gy;gd=math.hypot(gx,gy)
                        for px,py in self.points:
                            dx,dy=self.pose[0]-px,self.pose[1]-py;d=math.hypot(dx,dy)
                            gap=max(.01,d-self.memory.radius)
                            if .001<d and gap<.35:
                                weight=min(3.,.01*(1/gap-1/.35)/gap**2)*min(1.,(gd/.45)**2)
                                fx+=weight*dx/d/max(1,len(self.points))*8
                                fy+=weight*dy/d/max(1,len(self.points))*8
                        error=wrap(math.atan2(fy,fx)-self.pose[2])
                        w=clamp(1.5*error,-.25,.25)
                        v=.03*max(0.,math.cos(error))*min(1.,gd/.3) if abs(error)<.6 else 0.
                        if self.points and min(math.dist(self.pose[:2],p) for p in self.points)<=self.memory.radius+self.memory.margin:
                            v=w=0.;self.reason='CLEARANCE_STOP'
                        self.draw()
                    else:self.clear_markers()
            elif self.reason not in ('REACHED','MEMORY_RESET'):self.reason='IDLE_CLICK_GOAL'
        cmd=TwistStamped();cmd.header.frame_id='base_footprint';cmd.header.stamp=self.get_clock().now().to_msg()
        cmd.twist.linear.x=float(v);cmd.twist.angular.z=float(w);self.cmd_pub.publish(cmd)
        if now-self.last_log>2:
            self.get_logger().info(f'{self.reason}; remembered_cells={len(self.memory.cells)}; targets={len(self.route)}; preview v={v:.3f}, w={w:.3f}; NO MOTOR OUTPUT')
            self.last_log=now

    def draw(self):
        path=Path();path.header.frame_id='odom';path.header.stamp=self.get_clock().now().to_msg()
        for x,y in [self.pose[:2]]+self.route:
            p=PoseStamped();p.header=path.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;path.poses.append(p)
        self.path_pub.publish(path);self.clear_only_markers()
        markers=MarkerArray()
        for i,(x,y) in enumerate(self.route+[self.goal]):
            m=Marker();m.header=path.header;m.ns='live_targets';m.id=i;m.type=Marker.SPHERE;m.action=Marker.ADD
            m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=.04;m.pose.orientation.w=1.
            m.scale.x=m.scale.y=m.scale.z=.06;m.color.a=1.;m.color.r=1.;m.color.g=.5 if i<len(self.route) else 0.
            markers.markers.append(m)
        self.mark_pub.publish(markers)

    def clear_only_markers(self):
        m=Marker();m.action=Marker.DELETEALL;self.mark_pub.publish(MarkerArray(markers=[m]))


def main():
    rclpy.init();node=Live()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
