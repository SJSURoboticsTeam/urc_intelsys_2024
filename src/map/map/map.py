import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from constants import (
    MAP_TOPIC,
    QOS,
    COMPASS_TOPIC,
    CARTESIAN_TOPIC,
    DETECTION_TOPIC,
)
from std_msgs.msg import Float64
from urc_intelsys_2024_msgs.msg import CART
from std_msgs.msg import Float32MultiArray
import math


class MapNode(Node):
    def __init__(self):
        super().__init__("map")
        self.declare_parameters(
            "",
            [
                ("width", 100),
                ("height", 100),
                ("map_publish_seconds", 1.0),
                ("frame_id", "world"),
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

        # in order to track where we are so that we can properly
        # update the map when we receive detections

        # listen to COMPAS_TOPIC to get raw angle
        self.create_subscription(Float64, COMPASS_TOPIC, self.compass_callback, QOS)
        # self.create_subscription(Quaternion, COMPASS_TOPIC, self.compass_callback, QOS)

        self.create_subscription(CART, CARTESIAN_TOPIC, self.cart_callback, QOS)
        self.orientation = None
        self.cart = None
        # actually listen to detections
        self.create_subscription(
            Float32MultiArray, DETECTION_TOPIC, self.handle_detections, QOS
        )

    def compass_callback(self, orientation: Float64):
        self.orientation = orientation.data

    def cart_callback(self, cartesian: CART):
        self.cart = cartesian

    def handle_detections(self, detections: Float32MultiArray):
        stride = detections.layout.dim[1].stride
        num_detections = len(detections.data) // stride
        for i in range(num_detections):
            # place obstacles
            confidence, angle, height, width, distance = detections.data[
                i * stride : (i + 1) * stride
            ]
            # place the obstacles, based on current cart and orientation
            # combine current angle with the detection angle
            # both are in degrees, so we can just add
            # TODO - distance is in mm, not meters
            combined_angle = angle + self.orientation

            #scale confidence by 100
            confidence = confidence *100
            #convert distance, height, and width from mm to meters
            distance = distance/100
            height = height/100
            width = width/100

            self.get_logger().info(
                f"We have {confidence} {angle} {height} {width} {distance} and combined {combined_angle}"
            )
            # after that, we would use distance as the hypotenuse of a triangle and then
            # figure out what the distance in terms of x and y would be
            # TODO - account for height and width
            
            x_increment, y_increment = self.get_cartesian_distance(
                combined_angle, distance
            )
            # then finally, add those to our current cartesian position
            x_obstacle = self.cart.x + x_increment
            y_obstacle = self.cart.y + y_increment
            # mark that cell in our grid as an obstacle w/ {confidence} confidence
            self.get_logger().info("x: %s, y: %s" % (x_obstacle, y_obstacle))
            # TODO - scale confidence by 100
            #self.set_grid(x_obstacle, y_obstacle, confidence)
            #convert height and width from float to integers
            for i in range((int)(height)):
                for j in range((int)(width)):
                    self.set_grid(x_obstacle+j, y_obstacle+i, confidence)
            

    def set_grid(self, row: int | float, col: int | float, value: float):
        self.data[int(row) * self.width + int(col)] = int(value)

    def get_cartesian_distance(self, angle: float, distance: float):
        # convert from the polar (angle, distance) to the cartesian
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        return x, y

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
