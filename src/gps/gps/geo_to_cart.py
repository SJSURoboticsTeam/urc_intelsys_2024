import rclpy
from rclpy.node import Node
from constants import GPS_TOPIC, CARTESIAN_TOPIC
from urc_intelsys_2024_msgs.msg import GPS, CART
from urc_intelsys_2024_msgs.srv import GeoToCart


class CartesianPublisher(Node):
    def __init__(self):
        super().__init__("Cartesian_Publisher")
        self.publisher_ = self.create_publisher(CART, CARTESIAN_TOPIC, 10)

        # use the GeoToCart service
        self.cli = self.create_client(GeoToCart, 'geo_to_cart')
        self.req = GeoToCart.Request()

        # subscribe to gps message
        self.subscription = self.create_subscription(
            GPS, GPS_TOPIC, self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg: GPS):
        self.req.inp = msg
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.future_callback)

    def future_callback(self, future: rclpy.Future):
        response = future.result().out
        self.publisher_.publish(response)
        self.get_logger().info("Publishing x-coordinate: %s" % response.x)
        self.get_logger().info("Publishing y-coordinate: %s" % response.y)


def main(args=None):
    rclpy.init(args=args)

    try:
        rclpy.spin(CartesianPublisher())
    except KeyboardInterrupt:
        print("shutting down")
