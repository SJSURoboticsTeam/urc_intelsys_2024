import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from constants import MAP_TOPIC, QOS, DEFAULT_FRAME


class MapNode(Node):
    def __init__(self):
        super().__init__("map")
        self.declare_parameters(
            "",
            [
                ("width", 100),
                ("height", 100),
                ("map_publish_seconds", 1.0),
                ("frame_id", DEFAULT_FRAME),
            ],
        )

        self.publisher = self.create_publisher(OccupancyGrid, MAP_TOPIC, QOS)
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.frame_id = self.get_parameter("frame_id").value
        self.data = [0] * (self.width * self.height)
        self.data[0 * self.width + 1] = 0
        self.data[0 * self.width + 0] = 0
        self.data[0 * self.width + 2] = 100
        self.data[1 * self.width + 0] = 100
        self.data[1 * self.width + 1] = 50
        self.data[1 * self.width + 2] = 0

        self.publisher.publish(self.get_map())
        self.create_timer(
            self.get_parameter("map_publish_seconds").value,
            lambda: self.publisher.publish(self.get_map()),
        )

    def get_map(self):
        grid = OccupancyGrid()
        grid.data = self.data

        # set attributes
        grid.info.height = self.height
        grid.info.width = self.width
        grid.info.map_load_time = self.get_clock().now().to_msg()
        grid.info.resolution = 1.0
        grid.header.frame_id = self.frame_id
        grid.header.stamp = self.get_clock().now().to_msg()

        # send
        return grid


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(MapNode())
    except KeyboardInterrupt:
        print("Shutting down map")
