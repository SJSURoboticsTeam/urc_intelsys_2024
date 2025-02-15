import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid
from urc_intelsys_2024_msgs.msg import CART
from constants import MAP_TOPIC, GOAL_TOPIC, CARTESIAN_TOPIC, QOS
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point, PoseStamped

class PathfinderPublisher(Node):
    def __init__(self):
        super().__init__("pathfinder_publisher")

        self.create_subscription(OccupancyGrid, MAP_TOPIC, self.default_callback, 10)
        self.create_subscription(Float64, GOAL_TOPIC, self.default_callback, 10)
        self.create_subscription(CART, CARTESIAN_TOPIC,self.default_callback, 10)

        self.publisher_ = self.create_publisher(Path, "pathfinder_topic", 10)

    def default_callback(self, msg):
        #the actual pathfinding algorithm


        path = self.a_star(msg.data)
        path = Path()
        pose_stamped  = PoseStamped()
        temp = OccupancyGrid()
        self.publisher_.publish(path)

        self.get_logger().info(f'Publishing Path: "{path}"')

def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(PathfinderPublisher())
    except KeyboardInterrupt:
        print("Shutting down")
