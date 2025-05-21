import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import ComputePathToPose   # ← note .action, not .srv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path           # ← import Path
class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self._client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._client.wait_for_server()
        self.send_goal()
        self.path_pub = self.create_publisher(Path, 'plan', 10)

    def send_goal(self):
        goal_msg = ComputePathToPose.Goal()

        # Nav2 will use the robot’s current pose.
        goal_msg.start = PoseStamped()
        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.pose.position.x = -1.0
        goal_msg.start.pose.position.y = -1.0
        goal_msg.start.pose.orientation.w = 1.0
        goal_msg.use_start = True

        # Your desired goal
        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.pose.position.x = 1.5
        goal_msg.goal.pose.position.y = 1.0
        goal_msg.goal.pose.orientation.w = 1.0

        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        # fb.current_goal… fb.planning_time… etc.
        self.get_logger().info(f"Planning time so far: {fb.planning_time.sec}s")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted, waiting for result…')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        path = result.path           # nav_msgs/Path
        self.get_logger().info(f"Received path with {len(path.poses)} waypoints:")
        for i, p in enumerate(path.poses):
            x = p.pose.position.x; y = p.pose.position.y
            self.get_logger().info(f"  {i}: ({x:.2f}, {y:.2f})")
        self.path_pub.publish(path)    # ← republish into RViz
        rclpy.shutdown()

def main():
    rclpy.init()
    node = PathPlanner()
    rclpy.spin(node)
