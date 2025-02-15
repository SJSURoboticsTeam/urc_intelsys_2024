import rclpy.node
from constants import GPS_TOPIC, QOS, GOAL_TOPIC
from urc_intelsys_2024_msgs.msg import GPS, CART
from urc_intelsys_2024_msgs.srv import GPSDistance, GeoToCart


class GPSNav(rclpy.node.Node):
    """
    GPS Navigation node. Task details for this should look like the
    following:

    {"goal_latitude": float, "goal_longitude": float}
    """

    def __init__(self):
        super().__init__("gpsnav")
        self.create_subscription(GPS, GPS_TOPIC, self.update_cur_gps, QOS)

        self.cur_gps = None
        self.goal_gps = None
        self.goal_cart = None
        self.last_known_distance = float("inf")
        self.distance_client = self.create_client(GPSDistance, "gps_distance")
        self.cart_client = self.create_client(GeoToCart, "geo_to_cart")
        self.requests = set()

        self.goal_publisher = self.create_publisher(CART, GOAL_TOPIC, QOS)
        self.create_timer(1.0, self.publish_goal_cart)

    def publish_goal_cart(self):
        if self.goal_gps is not None and self.goal_cart is not None:
            self.goal_publisher.publish(self.goal_cart)

    def update_goal_cart(self, future: rclpy.Future):
        repsonse: GeoToCart.Response = future.result()
        self.goal_cart = repsonse.out

    def update_cur_gps(self, a: GPS):
        self.cur_gps = a

    def update_last_known_distance(self, future: rclpy.Future):
        self.requests.remove(future)
        response: GPSDistance.Response = future.result()
        self.last_known_distance = response.distance_in_km

    def call(self, goal_latitude: float, goal_longitude: float) -> bool:
        self.get_logger().info(
            f"called with {goal_latitude} {goal_longitude}, cur gps {self.cur_gps}",
        )

        goal_gps = GPS()
        goal_gps.latitude = goal_latitude
        goal_gps.longitude = goal_longitude
        if goal_gps != self.goal_gps:
            self.goal_cart = None
            self.goal_gps = goal_gps
            request = GeoToCart.Request()
            request.inp = goal_gps
            self.cart_client.call_async(request).add_done_callback(
                self.update_goal_cart
            )
        if self.cur_gps is not None:
            request = GPSDistance.Request()
            request.current = self.cur_gps
            request.target = goal_gps

            # call and add to list
            call = self.distance_client.call_async(request)
            call.add_done_callback(self.update_last_known_distance)
            self.requests.add(call)

            self.get_logger().info(
                f"Distance to target is {self.last_known_distance*1000}"
            )
            if self.last_known_distance * 1000 <= 2:
                # reset distance
                self.last_known_distance = float("inf")
                for call in self.requests:
                    call.cancel()
                self.requests = set()
                # reset goal gps and goal cart
                self.goal_gps = None
                self.goal_cart = None
                return True
        return False
