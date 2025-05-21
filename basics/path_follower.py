import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
import tf2_ros
import tf2_geometry_msgs  # for do_transform_pose
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Path           # ← import Path

import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        qos = QoSProfile(depth=10)
        self.path_sub = self.create_subscription(
            Path, '/path', self.path_callback, 10)
            
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.waypoints = []            # list of geometry_msgs/PoseStamped
        
        self.create_timer(0.1, self.control_loop)
        
        
    def path_callback(self, msg: Path):
        """Store the new path and reset waypoint index."""
        self.get_logger().info(f"Received path with {len(msg.poses)} points")
        self.waypoints = msg.poses
        self.current_goal_idx = 0   
        
    def control_loop(self):
        """Runs at a fixed rate, sending Twist commands toward the current waypoint."""
        if not self.waypoints:
            return  # no path to follow

        # pick the current target pose
        target_pose_map = self.waypoints[self.current_goal_idx]
        
        #print("Target pose map:", target_pose_map)
        
        try:
            # look up transform from map → base_link
            t: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link',
                target_pose_map.header.frame_id,  # usually "map"
                rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("TF not available yet")
            return

        # transform the target pose into the robot’s base_link frame
        target_pose_base = tf2_geometry_msgs.do_transform_pose(
            target_pose_map.pose, t)
        
        #print("Target pose base:", target_pose_base)
        
        # compute distance & angle to the waypoint in the robot frame
        dx = target_pose_base.position.x
        dy = target_pose_base.position.y
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        print("dx:", dx, "dy:", dy, "distance:", distance, "angle:", angle)
        
        # controller gains & thresholds
        ANGLE_TOLERANCE = 0.05   # rad ~3°
        DIST_TOLERANCE  = 0.05   # m
        K_ANG = 1.0             # rad/sec per rad error
        K_LIN = 0.5             # m/sec per meter error

        twist = Twist()

        # 1) Rotate in place if we’re not facing the waypoint
        if abs(angle) > ANGLE_TOLERANCE:
            twist.angular.z = K_ANG * angle
            twist.linear.x = 0.0

        # 2) Otherwise drive straight toward it
        elif distance > DIST_TOLERANCE:
            twist.linear.x = K_LIN * distance
            twist.angular.z = 0.0
        
        # 3) If we’re within both tolerances, advance to the next waypoint
        else:
            self.get_logger().info(
                f"Reached waypoint {self.current_goal_idx}: "
                f"({target_pose_map.pose.position.x:.2f}, "
                f"{target_pose_map.pose.position.y:.2f})")
            if self.current_goal_idx < len(self.waypoints) - 1:
                self.current_goal_idx += 1
            else:
                # final waypoint reached → stop and clear path
                self.get_logger().info("All waypoints reached. Stopping.")
                self.waypoints = []
                twist = Twist()  # zero velocities
        print("Publishing:", twist)
        # publish the command
        self.cmd_pub.publish(twist)     
         
def main():
    rclpy.init()
    print("Starting follower..")
    node = PathFollower()
    rclpy.spin(node)
