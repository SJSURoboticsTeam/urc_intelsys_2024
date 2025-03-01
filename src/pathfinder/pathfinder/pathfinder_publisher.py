# imports manage them + remove stuff from main - eliot
# get right message types for subscriptions, and proper callbacks for each - japji
# add type annotations to a* algorithm and make it work with the information we have - alex start
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from urc_intelsys_2024_msgs.msg import CART
from constants import MAP_TOPIC, GOAL_TOPIC, CARTESIAN_TOPIC, PATH_TOPIC, DEFAULT_FRAME
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq


class PathfinderPublisher(Node):
    def __init__(self):
        super().__init__("pathfinder_publisher")

        # every time we receive an occupancy, we want to run a* on it
        # if we have our current goal and current position
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self.map_callback, 10)
        self.create_subscription(CART, GOAL_TOPIC, self.goal_callback, 10)
        self.create_subscription(CART, CARTESIAN_TOPIC, self.start_callback, 10)

        self.publisher_ = self.create_publisher(Path, PATH_TOPIC, 10)
        self.occupancy_grid: OccupancyGrid = None
        self.goal: tuple[int, int] = None
        self.start: tuple[int, int] = None
        # for testing purposes
        # self.goal: tuple[int, int] = (0, 0)
        # self.start: tuple[int, int] = (30, 30)

    def map_callback(self, msg: OccupancyGrid):
        self.occupancy_grid = msg

        # now that we have an occupancy grid, check if we have start and goal
        # if so, call a_star
        if self.goal is not None and self.start is not None:
            path = self.a_star()
            self.publish_path(path)

    def goal_callback(self, msg: CART):
        self.goal = (msg.x, msg.y)  # Assume goal is in world coordinates

    def start_callback(self, msg: CART):
        self.start = (msg.x, msg.y)  # Assuming CART message has x, y coordinates

    def a_star(self):
        # double check all the types over here
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        grid = np.array(self.occupancy_grid.data, dtype=np.int8).reshape(
            (height, width)
        )

        # heuristic is based on manhattan distance
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        # initialize nodes to visit (open_set) and parents (came_from)
        open_set: list[tuple[int, tuple[int, int]]] = []  # (score, point)
        heapq.heappush(open_set, (0, self.start))
        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: heuristic(self.start, self.goal)}

        while open_set:
            # take top
            _, current = heapq.heappop(open_set)

            # reached goal
            if current == self.goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            # explore neighbors
            neighbors = [
                (current[0] + dx, current[1] + dy)
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
                if 0 <= current[0] + dx < width and 0 <= current[1] + dy < height
            ]
            for neighbor in neighbors:
                if grid[neighbor[1], neighbor[0]] > 50:  # Obstacle threshold
                    continue

                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(
                        neighbor, self.goal
                    )
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found

    def publish_path(self, path_points: list[tuple[int, int]]):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = DEFAULT_FRAME

        for point in path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = float(point[0])
            pose_stamped.pose.position.y = float(point[1])
            pose_stamped.pose.position.z = 0.0
            path_msg.poses.append(pose_stamped)

        self.publisher_.publish(path_msg)
        self.get_logger().info("Publishing Path")


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(PathfinderPublisher())
    except KeyboardInterrupt:
        print("Shutting down")
