#!/usr/bin/env python3
"""RUN ON RASPBERRY PI: command timeout independent of laptop/Wi-Fi."""
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from drive_safety import CommandLease

class Relay(Node):
    def __init__(self):
        super().__init__('apf_robot_drive_relay')
        model=Path('/proc/device-tree/model')
        if not model.exists() or 'Raspberry Pi' not in model.read_text():
            raise RuntimeError('Run drive_relay.py on the robot Raspberry Pi, not laptop')
        self.lease=CommandLease(.4)
        self.pub=self.create_publisher(TwistStamped,'/cmd_vel',10)
        self.ready=self.create_publisher(Bool,'/apf/drive_ready',10)
        self.create_subscription(TwistStamped,'/apf/drive_request',self.receive,1)
        self.create_timer(.05,self.tick)
        self.get_logger().info('Robot-local relay ready; defaults to ZERO. Input expires after 0.4 s.')
    def receive(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age=self.get_clock().now().nanoseconds/1e9-stamp
        self.lease.accept(msg.twist.linear.x,msg.twist.angular.z,time.monotonic(),age)
    def tick(self):
        exclusive=self.count_publishers('/cmd_vel')==1 and self.count_publishers('/apf/drive_request')<=1
        if not exclusive:self.lease.last=float('-inf')
        v,w=self.lease.output(time.monotonic()) if exclusive else (0.,0.)
        self.send(v,w);self.ready.publish(Bool(data=exclusive))
    def send(self,v,w):
        msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_footprint'
        msg.twist.linear.x=v;msg.twist.angular.z=w;self.pub.publish(msg)

def main():
    rclpy.init();node=None
    try:
        node=Relay();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:
            if rclpy.ok():node.send(0.,0.)
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
