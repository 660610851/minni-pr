"""Display recorded odometry trail in RViz; never publishes motor commands."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

class Trajectory(Node):
 def __init__(self):
  super().__init__('apf_trajectory_view')
  self.path=Path()
  self.pub=self.create_publisher(Path,'/apf/actual_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.sub=self.create_subscription(Odometry,'/odom',self.update,qos_profile_sensor_data)
  self.srv=self.create_service(Trigger,'/apf/clear_actual_path',self.clear)
  self.timer=self.create_timer(.5,self.publish)
  self.get_logger().info('Visualization only; RViz Path topic=/apf/actual_path. No motor output.')
 def clear(self,req,res):
  self.path.poses.clear();self.publish();res.success=True;res.message='Odometry trail cleared';return res
 def update(self,msg):
  if self.path.header.frame_id!=msg.header.frame_id:
   self.path=Path();self.path.header.frame_id=msg.header.frame_id
  if not msg.header.frame_id:return
  p=PoseStamped();p.header=msg.header;p.pose=msg.pose.pose
  if self.path.poses:
   a=self.path.poses[-1].pose.position;b=p.pose.position
   if math.hypot(a.x-b.x,a.y-b.y)<.01:return
  self.path.poses.append(p)
  if len(self.path.poses)>20000:self.path.poses=self.path.poses[-20000:]
 def publish(self):
  self.path.header.stamp=self.get_clock().now().to_msg();self.pub.publish(self.path)

def main():
 rclpy.init();n=Trajectory()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
