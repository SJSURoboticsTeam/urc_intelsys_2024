import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion
from constants import COMPASS_TOPIC, QOS
import math

class QuaternionPublisher(Node):
    def __init__(self):
        super().__init__('quaternion_publisher')

        self.create_subscription(Float64, COMPASS_TOPIC, self.float_callback, 10)
        
        self.publisher_ = self.create_publisher(Quaternion, 'quaternion_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def float_callback(self, msg):
        #convert the float to a quaternion
        quaternion = self.convert_float_to_quaternion(msg.data)

        self.publisher_.publish(quaternion)

        self.get_logger().info(f'Publishing Quaternion: "{quaternion}"')
    
    def convert_float_to_quaternion(self, angle):
        angle = math.radians(angle)
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(angle/2.0)
        q.w = 1-(q.z*q.z)
        return q
    
def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(QuaternionPublisher())
    except KeyboardInterrupt:
        print("Shutting down")