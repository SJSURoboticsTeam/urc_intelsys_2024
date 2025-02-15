import rclpy.node
from constants import GPS_TOPIC, QOS
from urc_intelsys_2024_msgs.msg import GPS
from urc_intelsys_2024_msgs.srv import GPSDistance


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
        self.last_known_distance = float("inf")
        self.client = self.create_client(GPSDistance, "gps_distance")
        self.requests = set()

    def update_cur_gps(self, a: GPS):
        self.cur_gps = a

    def update_last_known_distance(self, future: rclpy.Future):
        self.requests.remove(future)
        response: GPSDistance.Response = future.result()
        self.last_known_distance = response.distance_in_km

    def call(self, goal_latitude: float, goal_longitude: float) -> bool:
        self.get_logger().info(
            f"called with {goal_latitude} {goal_longitude} of type {type(goal_longitude)} {type(goal_latitude)}, cur gps {self.cur_gps}, self {self}",
        )
        goal_gps = GPS()
        goal_gps.latitude = goal_latitude
        goal_gps.longitude = goal_longitude
        if self.cur_gps is not None:
            request = GPSDistance.Request()
            request.current = self.cur_gps
            request.target = goal_gps

            # call and add to list
            call = self.client.call_async(request)
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
                return True
        return False
