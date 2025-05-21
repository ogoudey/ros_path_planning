import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')
        
        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE



        self.pub = self.create_publisher(OccupancyGrid, 'map', qos)
        # build a simple 100×100 grid
        data = np.zeros((100,100), dtype=np.int8)
        # draw a 20×20 obstacle in the centre
        data[40:60, 40:60] = 100

        # fill in the message
        self.msg = OccupancyGrid()
        self.msg.header.frame_id = 'map'
        self.msg.info = MapMetaData()
        self.msg.info.resolution = 0.05           # 5 cm per cell
        self.msg.info.width  = data.shape[1]
        self.msg.info.height = data.shape[0]
        # center the origin so map spans from −2.5 m to +2.5 m
        self.msg.info.origin.position.x = -data.shape[1] * 0.05 / 2.0
        self.msg.info.origin.position.y = -data.shape[0] * 0.05 / 2.0
        self.msg.data = data.flatten().tolist()

        # publish at 1 Hz
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)

def main():
    rclpy.init()
    node = OccupancyGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

