import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster

import math

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        
        self.x = -2.0
        self.y = -2.0
        self.yaw = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.last_time = self.get_clock().now()
        
        
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.pub = self.create_publisher(PoseStamped, 'robot_pose', 10)
        
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_timer(0.1, self.timer_callback)

            
    def cmd_callback(self, msg: Twist):
        print(msg)
        self.v = msg.linear.x
        self.omega = msg.angular.z
    
    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # simple unicycle integration
        dx = self.v * math.cos(self.yaw) * dt
        dy = self.v * math.sin(self.yaw) * dt
        dtheta = self.omega * dt

        self.x += dx
        self.y += dy
        self.yaw += dtheta
        print(self.yaw)
        try:
            # lookup transform from 'map' into 'base_link'
            t = self.tf_buffer.lookup_transform(
                'map',       # target frame
                'base_link', # source frame
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.orientation.z = math.sin(self.yaw/2)
        msg.pose.orientation.w = math.cos(self.yaw/2)

        self.pub.publish(msg)
        
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        # convert yaw → quaternion
        t.transform.rotation.z = math.sin(self.yaw/2.0)
        t.transform.rotation.w = math.cos(self.yaw/2.0)

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
