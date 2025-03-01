import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid
from urc_intelsys_2024_msgs.msg import CART
from constants import MAP_TOPIC, GOAL_TOPIC, CARTESIAN_TOPIC, QOS
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point, PoseStamped
import numpy as np

class PathfinderPublisher(Node):
    def __init__(self):
        super().__init__("pathfinder_publisher")

        self.create_subscription(OccupancyGrid, MAP_TOPIC, self.default_callback, 10)
        self.create_subscription(Float64, GOAL_TOPIC, self.default_callback, 10)
        self.create_subscription(CART, CARTESIAN_TOPIC,self.default_callback, 10)

        self.publisher_ = self.create_publisher(Path, "pathfinder_topic", 10)
        self.occupancy_grid = None
        self.goal = None
        self.start = None

    def default_callback(self, msg):
        #the actual pathfinding algorithm


        path = self.a_star(msg.data)
        path = Path()
        pose_stamped  = PoseStamped()
        temp = OccupancyGrid()
        self.publisher_.publish(path)

        self.get_logger().info(f'Publishing Path: "{path}"')
    def map_callback(self, msg):
        self.occupancy_grid = msg

    def goal_callback(self, msg):
        self.goal = msg.data  # Assume goal is in world coordinates

    def start_callback(self, msg):
        self.start = (msg.x, msg.y)  # Assuming CART message has x, y coordinates

        if self.occupancy_grid and self.goal:
            path = self.a_star(self.occupancy_grid, self.start, self.goal)
            self.publish_path(path)

    def a_star(self, occupancy_grid, start, goal):
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution
        origin = (occupancy_grid.info.origin.position.x, occupancy_grid.info.origin.position.y)
        grid = np.array(occupancy_grid.data).reshape((height, width))

        def world_to_grid(world_x, world_y):
            grid_x = int((world_x - origin[0]) / resolution)
            grid_y = int((world_y - origin[1]) / resolution)
            return grid_x, grid_y

        def grid_to_world(grid_x, grid_y):
            world_x = grid_x * resolution + origin[0]
            world_y = grid_y * resolution + origin[1]
            return world_x, world_y

        start_grid = world_to_grid(start[0], start[1])
        goal_grid = world_to_grid(goal[0], goal[1])

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: heuristic(start_grid, goal_grid)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_grid:
                path = []
                while current in came_from:
                    path.append(grid_to_world(*current))
                    current = came_from[current]
                path.reverse()
                return path

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
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found
    
    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for point in path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0
            path_msg.poses.append(pose_stamped)

        self.publisher_.publish(path_msg)
        self.get_logger().info("Publishing Path")
def generate_large_grid(size=4000, obstacle_prob=0.3):
        """
        Generates a large grid with obstacles.
        0 = free space, 1 = obstacle.
        """
        grid = np.random.choice([0, 1], size=(size, size), p=[1-obstacle_prob, obstacle_prob])
        return grid


size = 4000
grid = generate_large_grid(size=size, obstacle_prob=0.3)

start = (0, 0)
goal = (size-1, size-1)

print("Running optimized A* on a 4000x4000 grid...")
start_time = time.time()
path = a_star(grid, start, goal)
end_time = time.time()

if path:
    print(f"Path found! Length: {len(path)}")
else:
    print("No path found.")

def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(PathfinderPublisher())
    except KeyboardInterrupt:
        print("Shutting down")
