import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from constants import MAP_TOPIC, QOS


class MapNode(Node):
    def __init__(self):
        super().__init__("map")
        self.declare_parameters(
            "",
            [("width", 100), ("height", 100), ("map_publish_seconds", 1.0)],
        )

        self.publisher = self.create_publisher(OccupancyGrid, MAP_TOPIC, QOS)
        self.update_publisher = self.create_publisher(
            OccupancyGrid, MAP_TOPIC + "_updates", QOS
        )

        self.srv = self.create_service(GetMap, "map_service", self.cb)

        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.data = [-1] * (self.width * self.height)
        self.data[0 * self.width + 1] = 0
        self.data[0 * self.width + 0] = 0
        self.data[0 * self.width + 2] = 100
        self.data[1 * self.width + 0] = 100
        self.data[1 * self.width + 1] = 50
        self.data[1 * self.width + 2] = 0

        self.publisher.publish(self.get_map())
        self.create_timer(5, lambda: self.publisher.publish(self.get_map()))

    def get_map(self):
        grid = OccupancyGrid()
        grid.data = self.data

        # set attributes
        grid.info.height = self.height
        grid.info.width = self.width
        grid.info.map_load_time = self.get_clock().now().to_msg()
        grid.info.resolution = 1.0
        grid.header.frame_id = "oak"  # TODO maybe make this configurable
        grid.header.stamp = self.get_clock().now().to_msg()

        # send
        return grid

    def cb(self, request, response):
        response.map = self.get_map()
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(MapNode())
    except KeyboardInterrupt:
        print("Shutting down map")
