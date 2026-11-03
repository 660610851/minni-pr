#!/usr/bin/env python3
"""Motor-capable direct LiDAR APF. Disarmed until /apf/arm + NEW goal.
Shares timestamped sensing and robot-local relay; no occupancy-grid motion gate.
"""
import time
import json
from pathlib import Path
from datetime import datetime
import math
import rclpy
from std_msgs.msg import String
from rclpy.time import Time
from tf2_ros import TransformException
from live_preview import yaw
from live_controller import Control
from live_core import clamp
from drive_safety import MAX_LINEAR, MAX_ANGULAR
from reactive_core import Reactive,body_points,footprint_gap

class Direct(Control):
    def __init__(self):
        super().__init__()
        self.accumulate_memory=False
        self.declare_parameter('max_linear',MAX_LINEAR)
        self.declare_parameter('max_angular',MAX_ANGULAR)
        self.declare_parameter('goal_obstacle_clearance',.02)
        self.direct=Reactive(max_v=clamp(self.get_parameter('max_linear').value,0.,MAX_LINEAR),
                             max_w=clamp(self.get_parameter('max_angular').value,0.,MAX_ANGULAR),
                             margin=self.get_parameter('margin').value,
                             goal_obstacle_clearance=self.get_parameter('goal_obstacle_clearance').value)
        if not .02<=self.direct.margin<=.10:raise ValueError('margin must be 0.02..0.10 metres')
        if not math.isfinite(self.direct.goal_obstacle_clearance) or not .02<=self.direct.goal_obstacle_clearance<=.2:
            raise ValueError('goal_obstacle_clearance must be 0.02..0.20 metres')
        self.get_logger().info(f'Goal point exclusion={self.direct.goal_obstacle_clearance:.3f}m; footprint margin={self.direct.margin:.3f}m')
        logdir=Path(__file__).parent/'trials';logdir.mkdir(exist_ok=True)
        self.trial_file=(logdir/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.jsonl')).open('x',buffering=1)
        self.previous_control_pose=None;self.previous_control_time=None
        self.get_logger().info(f'TRIAL_LOG={self.trial_file.name}')
        self.get_logger().info(f'DIRECT_LIDAR_APF: max_linear={self.direct.max_v:.3f} m/s, max_angular={self.direct.max_w:.3f} rad/s; no grid gate; starts DISARMED.')
    def arm(self,req,res):
        now=time.monotonic()
        if not self.relay_ready or now-self.relay_time>.3:
            res.success=False;res.message='No fresh robot relay; keep drive_relay running on Raspberry Pi'
        elif self.pose is None or now-self.last_scan>.4:
            res.success=False;res.message='Wait for fresh timestamp-matched scan and TF: '+self.sensor_diagnostic()
        elif not self.points:
            res.success=False;res.message='No valid LiDAR obstacle returns'
        else:
            gap=min(footprint_gap(*p) for p in body_points(self.pose,self.points))
            if gap<=self.direct.margin:
                res.success=False;res.message=f'Measured footprint gap={gap:.3f}m, required>{self.direct.margin:.3f}m'
            else:
                self.direct.stop('ARMED_WAIT_GOAL');self.goal=None;self.route=[]
                self.armed=True;self.latched_fault=None;self.send()
                res.success=True;res.message=f'ARMED_WAIT_GOAL; measured footprint gap={gap:.3f}m; select NEW nearby odom goal'
        self.get_logger().info(res.message);return res
    def new_goal(self,msg):
        if not self.armed:self.get_logger().warn('DISARMED: call /apf/arm first');return
        if msg.header.frame_id!='odom':self.get_logger().warn('Goal frame must be odom');return
        p=(msg.pose.position.x,msg.pose.position.y)
        if not all(math.isfinite(a) for a in p):return
        self.direct.set_goal(p,time.monotonic());self.goal=p;self.route=[];self.latched_fault=None
        self.get_logger().info(f'Goal accepted {p}; DIRECT_LIDAR_APF MOTOR CONTROL')
    def fault(self,reason):
        if hasattr(self,'trial_file'):
            try:self.trial_file.write(json.dumps(dict(event='STOP',reason=reason,pose=self.pose,goal=self.goal,monotonic=time.monotonic()))+'\n')
            except OSError:pass
        if hasattr(self,'direct'):self.direct.stop(reason)
        super().fault(reason)
    def tick(self):
        tick_started=time.monotonic()
        now=tick_started;v=w=0.
        if self.armed:
            if not self.relay_ready or now-self.relay_time>.3:self.fault('RELAY_NOT_READY')
            elif self.pose is None or now-self.last_scan>.4:
                self.get_logger().warn('Sensor stop detail: '+self.sensor_diagnostic())
                self.fault('STALE_SCAN_OR_TF')
            elif self.direct.goal is not None:
                try:
                    tr=self.tf.lookup_transform('odom','base_footprint',Time())
                    tfage=(self.get_clock().now()-Time.from_msg(tr.header.stamp)).nanoseconds/1e9
                    if not -.1<=tfage<=.4:raise ValueError(f'CONTROL_POSE_AGE={tfage:.3f}s')
                    self.pose=(tr.transform.translation.x,tr.transform.translation.y,yaw(tr.transform.rotation))
                except (TransformException,ValueError) as e:
                    self.fault(str(e));return
                previous_temp=self.direct.temp
                previous_goal=self.direct.goal
                v,w=self.direct.step(self.pose,self.points,now,data_age=max(0.,now-self.last_scan));self.reason=self.direct.state
                if self.direct.temp is not None and self.direct.temp!=previous_temp:
                    self.get_logger().info(f'TEMP_SELECTED: {self.direct.temp}; attempts={self.direct.attempts}')
                if self.direct.state=='REACHED':
                    self.get_logger().info(f'REACHED_BY_ODOMETRY pose={self.pose[:2]}, goal={previous_goal}, distance={math.dist(self.pose[:2],previous_goal):.3f}m')
                if self.direct.goal is None:self.fault(self.direct.state);v=w=0.
                else:
                    self.goal=self.direct.goal;self.route=[self.direct.temp or self.direct.goal];self.draw()
            else:self.reason='ARMED_WAIT_GOAL'
        else:
            self.reason=self.latched_fault or 'DISARMED'
        if not self.armed:v=w=0.
        self.send(v,w)
        if hasattr(self,'trial_file'):
            record=dict(monotonic=now,pose=self.pose,goal=self.direct.goal,temp=self.direct.temp,
                        state=self.reason,armed=self.armed,request_v=v,request_w=w,
                        max_linear=self.direct.max_v,accepted_age=(now-self.last_scan if self.last_scan else None),
                        points=self.points)
            try:self.trial_file.write(json.dumps(record,allow_nan=False)+'\n')
            except (OSError,ValueError) as e:
                if self.armed:self.fault('TRIAL_LOG_FAILURE: '+str(e))
        control_ms=(time.monotonic()-tick_started)*1000
        distance='none' if self.pose is None or self.direct.goal is None else f'{math.dist(self.pose[:2],self.direct.goal):.3f}m'
        text=f'DIRECT_APF goal_distance={distance}; armed={self.armed}; {self.reason}; v={v:.3f}, w={w:.3f}; no grid gate'
        self.status.publish(String(data=text))
        if now-self.last_drive_log>2:
            self.get_logger().info(text+f'; hold={getattr(self.direct, "hold_reason", "NONE")}; heading_error_deg={math.degrees(getattr(self.direct, "heading_error", 0.)):.1f}; control_ms={control_ms:.1f}; '+self.sensor_diagnostic());self.last_drive_log=now

def main():
    rclpy.init();node=None
    try:
        node=Direct();rclpy.spin(node)
    except KeyboardInterrupt:pass
    except Exception:
        if node is not None and rclpy.ok():node.send()
        raise
    finally:
        if node is not None:
            if rclpy.ok():node.send()
            if hasattr(node,'trial_file'):node.trial_file.close()
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
