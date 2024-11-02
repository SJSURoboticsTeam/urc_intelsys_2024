import rclpy
from rclpy.node import Node
from constants import GPS_TOPIC, CARTESIAN_TOPIC
from std_msgs.msg import String
from urc_intelsys_2024_msgs.msg import GPS

class CartesianPublisher(Node):
    def __init__(self):
        super().__init__('Cartesian Publisher')
        self.publisher_ = self.create_publisher(String, CARTESIAN_TOPIC, 10)
        self.subscription = self.create_subscription(
                GPS,
                GPS_TOPIC,
                self.listener_callback,
                10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    my_node = CartesianPublisher()

    rclpy.spin(my_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()