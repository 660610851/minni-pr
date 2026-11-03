#!/usr/bin/env python3
"""Live APF requests motor output ONLY after arm and a new goal.
Requires robot-local drive_relay.py. Experimental, no U escape guarantee.
"""
import math
import time
from collections import deque
import rclpy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool,String
from std_srvs.srv import Trigger
from live_preview import Live
from live_core import swept_safe

class Capture:
    def __init__(self,publisher):self.publisher=publisher;self.msg=None
    def publish(self,msg):self.msg=msg;self.publisher.publish(msg)

class Control(Live):
    def __init__(self):
        super().__init__('apf_live_controller')
        self.cmd_pub=Capture(self.cmd_pub)
        self.drive=self.create_publisher(TwistStamped,'/apf/drive_request',1)
        self.status=self.create_publisher(String,'/apf/control_status',10)
        self.armed=False;self.relay_ready=False;self.relay_time=0.;self.started=None;self.latched_fault=None
        self.history=deque();self.attempts=0;self.last_visit=0.;self.last_drive_log=0.
        self.create_subscription(Bool,'/apf/drive_ready',self.ready,10)
        self.create_service(Trigger,'/apf/arm',self.arm)
        self.create_service(Trigger,'/apf/stop',self.stop_service)
        self.get_logger().info('DISARMED. Requires robot relay, then /apf/arm and NEW goal. Speed limits are printed by the selected controller.')
    def ready(self,msg):self.relay_ready=msg.data;self.relay_time=time.monotonic()
    def send(self,v=0.,w=0.):
        msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_footprint'
        msg.twist.linear.x=float(v);msg.twist.angular.z=float(w);self.drive.publish(msg)
    def fault(self,reason):
        self.armed=False;self.goal=None;self.route=[];self.history.clear();self.started=None
        self.send();self.clear_markers();self.reason=reason;self.latched_fault=reason
        self.get_logger().warn(reason+'; DISARMED; re-arm then choose a new goal')
    def arm(self,req,res):
        now=time.monotonic()
        if not self.relay_ready or now-self.relay_time>.3:
            res.success=False;res.message='No fresh exclusive robot relay; start relay on Raspberry Pi and stop other controllers'
        elif self.pose is None or now-self.last_scan>.4:
            res.success=False
            age='never' if self.last_scan==0 else f'{now-self.last_scan:.3f}s'
            res.message=(f'Wait for fresh scan + TF: accepted_scan_age={age}; '
                         f'pose_received={self.pose is not None}; pending_scans={len(self.pending.items)}; '
                         f'status={self.reason}')
        elif not swept_safe(self.memory,self.pose,0.,0.,now):
            res.success=False;res.message='Start clearance rejected: '+self.memory.start_diagnostic(self.pose,now)
        else:
            self.goal=None;self.route=[];self.started=None;self.history.clear();self.armed=True;self.latched_fault=None
            self.send();res.success=True;res.message='ARMED_WAIT_GOAL: select a NEW nearby odom goal'
        self.get_logger().info(res.message);return res
    def stop_service(self,req,res):
        self.fault('USER_STOP');res.success=True;res.message='Stopped and disarmed';return res
    def reset_memory(self,msg):
        self.fault('MEMORY_RESET');super().reset_memory(msg)
    def new_goal(self,msg):
        if not self.armed:
            self.get_logger().warn('Goal ignored: DISARMED. Call /apf/arm first.');return
        super().new_goal(msg)
        if self.reason=='GOAL_ACCEPTED':
            self.started=time.monotonic();self.history.clear();self.attempts=0
    def tick(self):
        # Parent callback computes only the preview command and online route.
        super().tick()
        now=time.monotonic();v=w=0.
        if self.armed:
            if not self.relay_ready or now-self.relay_time>.3:
                self.fault('RELAY_NOT_READY')
            elif now-self.last_scan>.4:
                self.fault('STALE_SCAN_OR_TF')
            elif self.started is not None and now-self.started>120:
                self.fault('GOAL_TIME_LIMIT')
            elif self.goal is not None:
                if not self.route:
                    detail=('; '+self.memory.start_diagnostic(self.pose,now)) if self.reason=='START_CLEARANCE' else ''
                    self.fault('NO_OBSERVED_ROUTE: '+self.reason+detail)
                elif self.reason=='CLEARANCE_STOP':self.fault('CLEARANCE_STOP')
                else:
                    target=self.route[0]
                    self.history.append((now,self.pose[:2],math.dist(self.pose[:2],target),target))
                    while self.history and now-self.history[0][0]>6.:self.history.popleft()
                    stalled=(len(self.history)>2 and now-self.history[0][0]>5.5 and
                             math.dist(target,self.history[0][3])<.10 and
                             max(math.dist(self.pose[:2],p) for _,p,_,_ in self.history)<.08 and
                             self.history[0][2]-math.dist(self.pose[:2],target)<.025)
                    if now-self.last_visit>.5:
                        for k in list(self.memory.visits):
                            if math.dist(((k[0]+.5)*self.memory.res,(k[1]+.5)*self.memory.res),self.pose[:2])<.20:
                                self.memory.visits[k]+=1
                        k=self.memory.key(*self.pose[:2]);self.memory.visits[k]=self.memory.visits.get(k,0)+1
                        self.last_visit=now
                    if stalled:
                        self.failed.append(target);self.route=[];self.history.clear();self.attempts+=1
                        self.reason='RESELECT_TEMPORARY_GOAL'
                        if self.attempts>=6:self.fault('RECOVERY_ATTEMPT_LIMIT')
                    elif self.cmd_pub.msg is not None:
                        v=self.cmd_pub.msg.twist.linear.x;w=self.cmd_pub.msg.twist.angular.z
                        if not swept_safe(self.memory,self.pose,v,w,now):
                            v=w=0.;self.route=[];self.reason='PREDICTED_PATH_BLOCKED'
            elif self.reason=='REACHED':
                self.fault('REACHED')
        if not self.armed:
            v=w=0.
            if self.latched_fault:self.reason=self.latched_fault
        self.send(v,w)
        text=f'armed={self.armed}; {self.reason}; MOTOR_REQUEST v={v:.3f}, w={w:.3f}'
        self.status.publish(String(data=text))
        if now-self.last_drive_log>2:self.get_logger().info(text);self.last_drive_log=now

def main():
    rclpy.init();node=None
    try:
        node=Control();rclpy.spin(node)
    except KeyboardInterrupt:pass
    except Exception:
        if node is not None and rclpy.ok():node.send()
        raise
    finally:
        if node is not None:
            if rclpy.ok():node.send()
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
