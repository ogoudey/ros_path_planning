import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import ComputePathToPose   # ← note .action, not .srv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path           # ← import Path
class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        
        self.current_pose: PoseStamped | None = None
        self.create_subscription(
            PoseStamped,
            'robot_pose',
            self.pose_callback,
            10)
            
        self._client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._client.wait_for_server()
        
        self.create_timer(0.1, self.send_goal)
        self.path_pub = self.create_publisher(Path, 'path', 10)

    def pose_callback(self, msg: PoseStamped):
        # Always keep the latest robot_pose
        print("Received pose:", msg)
        self.current_pose = msg

    def send_goal(self):
        if self.current_pose is None:
            return
        goal_positions = [2,2] #input("Goal coords: ").split(',')
        print("Input x:", goal_positions[0], "y:", goal_positions[1])
        print("Current pose:", self.current_pose.pose.position)
        goal_msg = ComputePathToPose.Goal()

        
        #goal_msg.start = self.current_pose
        goal_msg.start = PoseStamped()
        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.header.stamp = self.get_clock().now().to_msg()
        goal_msg.start.pose.position.x = self.current_pose.pose.position.x
        goal_msg.start.pose.position.y = self.current_pose.pose.position.y
        goal_msg.start.pose.orientation = self.current_pose.pose.orientation
        goal_msg.use_start = True
        
        
        goal_msg.use_start = True
        # Your desired goal
        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.pose.position.x = float(goal_positions[0])
        goal_msg.goal.pose.position.y = float(goal_positions[1])
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
            # Manually set frame_id
            path.poses[i].header.frame_id='map'
        self.path_pub.publish(path)    # ← republish into RViz
        rclpy.shutdown()

def main():
    rclpy.init()
    node = PathPlanner()
    rclpy.spin(node)
