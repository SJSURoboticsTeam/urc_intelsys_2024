import sys

import rclpy
from rclpy.node import Node

import numpy as np

from urc_intelsys_2024_msgs.msg import HexMap1

print(f"Using python at {sys.executable}")

def get_obstacles() -> np.ndarray:
    x = np.random.randint(0, 11, 10)
    y = np.random.randint(0, 5, x.shape)*2
    coords = np.stack([x,y+x%2], axis=1)-5
    return coords

class DummyMapPublisher(Node):
    def __init__(self):
        super().__init__("DummyMapPublisher")
        self.publisher_ = self.create_publisher(HexMap1, '/map', 10)
        self.timer = self.create_timer(1,  self.publish)

    
    def publish(self):
        map = HexMap1()
        map.flatcoords = get_obstacles().flatten().tolist()
        map.safe = False
        self.publisher_.publish(map)
        self.get_logger().info("Published")

def main(args=None):
    rclpy.init(args=args)
    dummy = DummyMapPublisher()
    rclpy.spin(dummy)        
    dummy.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()