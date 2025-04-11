import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from std_msgs.msg import Float32MultiArray, Float64
from urc_intelsys_2024_msgs.msg import GPS
from nav_msgs.msg import OccupancyGrid
from urc_intelsys_2024_msgs.srv import GeoToCart
import numpy as np
import math
import os
import time
from typing import Tuple, List, Optional
from matplotlib import pyplot as plt
from matplotlib import colors
from constants import COMPASS_TOPIC, GPS_TOPIC, DETECTION_TOPIC, MAP_TOPIC, QOS


class ObstacleMapper(Node):
    """Node that maps obstacles detected by the camera to the occupancy grid.

    Subscribes to compass, GPS, and detection topics to update a map with obstacles.
    Publishes the updated map with obstacles.
    """

    # Maximum value for obstacle cells
    OBSTACLE_VALUE = 100

    def __init__(self) -> None:
        """Initialize the ObstacleMapper node."""
        super().__init__("obstacle_mapper")

        # Parameters
        self.declare_parameters(
            "",
            [
                ("update_frequency", 10.0),  # Hz
                (
                    "max_recent_detections",
                    10,
                ),  # Maximum number of recent detections to keep track of
                (
                    "max_displayed_detections",
                    3,
                ),  # Maximum number of closest detections to display coordinates for
                ("debug_save_maps", True),  # set to False to disable map saving
                (
                    "debug_save_maps_interval",
                    10.0,
                ),  # how often to save maps, in seconds
                (
                    "debug_save_maps_max_maps",
                    10,
                ),  # max number of maps to have saved at a time
                ("debug_save_maps_save_dir", "./map_images/"),  # directory to save maps
            ],
        )

        # Current position and orientation
        self.current_latitude: Optional[float] = None
        self.current_longitude: Optional[float] = None
        self.current_heading: Optional[float] = None

        # Map data
        self.map: Optional[OccupancyGrid] = None
        self.map_origin_x: float = 0
        self.map_origin_y: float = 0
        # Initialize map properties
        self.map_resolution: float = 1.0  # Will be updated when map is received
        self.map_width: int = 0
        self.map_height: int = 0

        # Track recent detections for visualization
        self.recent_detections: List[dict] = []
        self.max_recent_detections: int = self.get_parameter(
            "max_recent_detections"
        ).value

        # Create QoS profile for map publisher
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Define topics
        self.MAP_WITH_OBSTACLES_TOPIC = "/map_with_obstacles"
        self.CLOSEST_DETECTIONS_TOPIC = "/closest_detections"

        # Subscribers
        self.compass_sub = self.create_subscription(
            Float64, COMPASS_TOPIC, self.compass_callback, QOS
        )

        self.gps_sub = self.create_subscription(GPS, GPS_TOPIC, self.gps_callback, QOS)

        self.detection_sub = self.create_subscription(
            Float32MultiArray, DETECTION_TOPIC, self.detection_callback, QOS
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, MAP_TOPIC, self.map_callback, QOS
        )

        # Publishers
        self.map_publisher = self.create_publisher(
            OccupancyGrid, self.MAP_WITH_OBSTACLES_TOPIC, map_qos
        )

        self.closest_detections_publisher = self.create_publisher(
            Float32MultiArray, self.CLOSEST_DETECTIONS_TOPIC, QOS
        )

        self.get_logger().info(
            f"Created map publisher on topic: {self.MAP_WITH_OBSTACLES_TOPIC}"
        )

        # Create client for GeoToCart service
        self.geo_to_cart_client = self.create_client(GeoToCart, "geo_to_cart")

        # Create timer to periodically republish the map
        self.republish_timer = self.create_timer(1.0, self.republish_map)

        # Initialize default map
        self.init_default_map()

        # Track compass data reception
        self.last_compass_time = None
        self.compass_received = False

        # Initialize map visualization and saving
        self.debug_save_maps = self.get_parameter("debug_save_maps").value
        if self.debug_save_maps:
            self.save_dir = self.get_parameter("debug_save_maps_save_dir").value

            # Check if save_dir is empty and provide a default
            if not self.save_dir:
                from ament_index_python.packages import get_package_share_directory

                package_share_dir = get_package_share_directory("obstacle_detection")
                self.save_dir = os.path.join(package_share_dir, "map_images")
                self.get_logger().warning(
                    f"No save directory provided for maps. Using default: {self.save_dir}"
                )

            os.makedirs(self.save_dir, exist_ok=True)
            self.last_save_time = time.time()
            self.save_interval = self.get_parameter("debug_save_maps_interval").value
            self.max_maps = self.get_parameter("debug_save_maps_max_maps").value

            # Create timer for map saving
            self.save_timer = self.create_timer(1.0, self.check_save_map)

            self.get_logger().info(
                f"Map saving enabled. Maps will be saved to {self.save_dir}"
            )

        self.get_logger().info("ObstacleMapper initialized")

    def init_default_map(self) -> None:
        """Initialize a default map if none is provided."""
        if self.map is None:
            width = 200
            height = 200
            resolution = 0.1  # 0.1 meters per cell for finer detail

            # Create map message
            default_map = OccupancyGrid()
            default_map.header.stamp = self.get_clock().now().to_msg()
            default_map.header.frame_id = "world"

            default_map.info.width = width
            default_map.info.height = height
            default_map.info.resolution = resolution
            default_map.info.origin.position.x = -width * resolution / 2
            default_map.info.origin.position.y = -height * resolution / 2
            default_map.info.origin.position.z = 0.0
            default_map.info.origin.orientation.w = 1.0

            # Initialize all cells to unknown (-1)
            default_map.data = [-1] * (width * height)

            # Set center region to free space (0)
            center_size = 20
            center_x = width // 2
            center_y = height // 2
            for y in range(center_y - center_size // 2, center_y + center_size // 2):
                for x in range(
                    center_x - center_size // 2, center_x + center_size // 2
                ):
                    if 0 <= x < width and 0 <= y < height:
                        index = y * width + x
                        default_map.data[index] = 0

            # Store map
            self.map = default_map
            self.map_width = width
            self.map_height = height
            self.map_resolution = resolution
            self.map_origin_x = default_map.info.origin.position.x
            self.map_origin_y = default_map.info.origin.position.y

            self.get_logger().info(
                f"Initialized default map: {width}x{height}, resolution: {resolution}"
            )

    def get_closest_detections(self, max_count: int = None) -> list:
        """Get the closest detections to the robot.

        Args:
            max_count: Maximum number of detections to return. If None, uses the max_displayed_detections parameter.

        Returns:
            List of detections sorted by distance to robot, limited to max_count.
        """
        if not self.recent_detections:
            return []

        # If max_count is not provided, use the parameter
        if max_count is None:
            max_count = self.get_parameter("max_displayed_detections").value

        # Get robot position
        robot_x, robot_y = self.get_robot_map_position()

        # Calculate distance from robot for each detection
        for detection in self.recent_detections:
            detection["distance_to_robot"] = math.sqrt(
                (detection["x"] - robot_x) ** 2 + (detection["y"] - robot_y) ** 2
            )

        # Sort detections by distance to robot and limit to max_count
        return sorted(self.recent_detections, key=lambda d: d["distance_to_robot"])[
            :max_count
        ]

    def republish_map(self) -> None:
        """Periodically republish map"""
        # Initialize a default map if needed
        if self.map is None:
            self.get_logger().info("No map available, initializing default")
            self.init_default_map()
            return

        # Create new map message
        updated_map = OccupancyGrid()
        updated_map.header = self.map.header
        updated_map.header.stamp = self.get_clock().now().to_msg()

        # Ensure frame_id is set
        if not updated_map.header.frame_id:
            updated_map.header.frame_id = "world"

        updated_map.info = self.map.info
        updated_map.data = self.map.data

        # Count cells for debugging
        occupied_cells = sum(1 for x in updated_map.data if x > 0)
        unknown_cells = sum(1 for x in updated_map.data if x == -1)
        free_cells = sum(1 for x in updated_map.data if x == 0)

        # Publish map
        self.map_publisher.publish(updated_map)

        # Publish closest detections
        self.publish_closest_detections()

    def publish_closest_detections(self) -> None:
        """Publish the closest detections to a separate topic."""
        # Get the closest detections
        closest_detections = self.get_closest_detections()

        if not closest_detections:
            return

        # Create a Float32MultiArray message
        msg = Float32MultiArray()

        # Format: [num_detections, det1_x, det1_y, det1_distance, det2_x, det2_y, det2_distance, ...]
        data = [float(len(closest_detections))]

        for detection in closest_detections:
            # Convert to grid coordinates for easier use by subscribers
            grid_x = int((detection["x"] - self.map_origin_x) / self.map_resolution)
            grid_y = int((detection["y"] - self.map_origin_y) / self.map_resolution)

            # Add detection data to the message
            data.extend(
                [float(grid_x), float(grid_y), float(detection["distance_to_robot"])]
            )

        msg.data = data

        # Publish the message
        self.closest_detections_publisher.publish(msg)
        self.get_logger().debug(
            f"Published {len(closest_detections)} closest detections"
        )

    def compass_callback(self, msg: Float64) -> None:
        """Update current heading from compass data.

        Args:
            msg: Compass message containing heading in degrees
        """
        old_heading = self.current_heading
        self.current_heading = msg.data

        # Update compass tracking
        current_time = self.get_clock().now()
        if self.last_compass_time is not None:
            time_diff = (
                current_time.nanoseconds - self.last_compass_time.nanoseconds
            ) / 1e9  # To seconds
        else:
            time_diff = 0.0
        self.last_compass_time = current_time
        self.compass_received = True

        # Log detailed compass information
        if old_heading is not None:
            self.get_logger().info(
                f"Compass update: current={self.current_heading:.2f}°, "
                f"previous={old_heading:.2f}°, "
                f"change={abs(self.current_heading - old_heading):.2f}°, "
                f"time_since_last={time_diff:.3f}s"
            )
        else:
            self.get_logger().info(
                f"Compass update: current={self.current_heading:.2f}°, "
                f"previous=None, "
                f"change=N/A, "
                f"time_since_last={time_diff:.3f}s"
            )

    def gps_callback(self, msg: GPS) -> None:
        """Update current position from GPS data.

        Args:
            msg: GPS message containing latitude and longitude
        """
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        self.get_logger().debug(
            f"Current position: {self.current_latitude}, {self.current_longitude}"
        )

    def map_callback(self, msg: OccupancyGrid) -> None:
        """Update local map data from map service.

        Args:
            msg: OccupancyGrid message containing the updated map
        """
        # Check if map has valid dimensions
        if msg.info.width <= 0 or msg.info.height <= 0:
            self.get_logger().warn(
                f"Received map with invalid dimensions: {msg.info.width}x{msg.info.height}"
            )
            return

        # Check if this is the first map reception or if map dimensions changed
        is_first_map = self.map is None
        map_changed = self.map is not None and (
            self.map_width != msg.info.width
            or self.map_height != msg.info.height
            or self.map_resolution != msg.info.resolution
        )

        # Only update the entire map data if this is the first map or if dimensions changed
        if is_first_map or map_changed:
            self.get_logger().info(
                "Received new map or map dimensions changed, updating map data"
            )
            self.map = msg
            self.map_width = msg.info.width
            self.map_height = msg.info.height
            self.map_resolution = msg.info.resolution
            self.map_origin_x = msg.info.origin.position.x
            self.map_origin_y = msg.info.origin.position.y
        else:
            # Only update map metadata, keeping our obstacle data
            self.get_logger().debug(
                "Received map update, keeping obstacle data and updating metadata only"
            )
            self.map.header = msg.header
            self.map.info = msg.info

        # Count cells for debugging
        occupied_cells = sum(1 for x in self.map.data if x > 0)
        unknown_cells = sum(1 for x in self.map.data if x == -1)
        free_cells = sum(1 for x in self.map.data if x == 0)

        self.get_logger().info(
            f"Map updated: {self.map_width}x{self.map_height}, resolution: {self.map_resolution}, "
            f"cells: {len(self.map.data)}, occupied: {occupied_cells}, free: {free_cells}, unknown: {unknown_cells}"
        )

    def detection_callback(self, msg: Float32MultiArray) -> None:
        """Process detection messages and update the map with obstacle locations.

        Expected format: [angle, height, distance, spatial_x, spatial_y, spatial_z, confidence, xmin, xmax, ymin, ymax]

        Args:
            msg: Detection message containing obstacle data
        """
        # Check if map is available
        if self.map is None:
            self.get_logger().warn("Missing map data")
            return

        # Extract detection data
        if len(msg.data) < 3:
            self.get_logger().warn(f"Unexpected detection data format: {msg.data}")
            return

        # Basic data (always available)
        angle = msg.data[0]  # Angle in degrees relative to camera
        height = msg.data[1]  # Height in mm
        distance = msg.data[2]  # Distance in mm

        # Check if we have the extended spatial data
        has_spatial_data = len(msg.data) >= 7

        if has_spatial_data:
            spatial_x = msg.data[3]  # X coordinate in mm
            spatial_y = msg.data[4]  # Y coordinate in mm
            spatial_z = msg.data[5]  # Z coordinate in mm
            confidence = msg.data[6]  # Detection confidence

            self.get_logger().info(
                f"Detection: angle={angle:.1f}°, height={height:.1f}mm, distance={distance:.1f}mm, "
                f"spatial=({spatial_x:.1f}, {spatial_y:.1f}, {spatial_z:.1f})mm, confidence={confidence:.2f}"
            )
        else:
            self.get_logger().info(
                f"Detection: angle={angle:.1f}°, height={height:.1f}mm, distance={distance:.1f}mm"
            )

        # Convert to meters
        height_m = height / 1000.0
        distance_m = distance / 1000.0

        # Check if we've received compass data
        if not self.compass_received:
            self.get_logger().warn(
                "No compass data received yet, assuming heading of 0 degrees (North)"
            )

        # Use current heading if available, otherwise assume 0 (facing north)
        current_heading = 0.0 if self.current_heading is None else self.current_heading
        self.get_logger().info(
            f"Using heading: {current_heading}° for detection processing"
        )

        # Calculate global angle and convert to radians
        # Camera angle is relative to camera's forward direction
        # Positive angle means object is to the right, negative means to the left

        # First, convert camera-relative angle to robot-relative angle
        # In robot frame: 0° is forward, 90° is right, 180° is backward, 270° is left
        robot_relative_angle = angle  # Camera is assumed to be facing forward

        # Then convert to global angle using robot's heading
        # Global frame: 0° is East (positive x), 90° is North (positive y), etc.
        global_angle = (current_heading + robot_relative_angle) % 360
        global_angle_rad = math.radians(global_angle)

        self.get_logger().debug(
            f"Robot heading: {current_heading}°, Camera angle: {angle}°, Global angle: {global_angle}°"
        )

        # Get robot position in map coordinates
        robot_x, robot_y = self.get_robot_map_position()

        # Calculate obstacle position relative to robot in meters
        if has_spatial_data:
            # Use direct spatial coordinates when available (more accurate)
            # Convert from mm to meters and apply rotation based on robot heading
            spatial_x_m = spatial_x / 1000.0
            spatial_y_m = spatial_y / 1000.0
            spatial_z_m = spatial_z / 1000.0

            # Apply rotation based on robot heading
            # The spatial coordinates are in camera frame, we need to rotate them to world frame
            heading_rad = math.radians(current_heading)

            # Rotate the coordinates
            # In camera frame: +Z is forward, +X is right, +Y is down
            # We need to convert to world frame where: North is +Y, East is +X

            # First convert camera frame to robot frame
            robot_frame_x = spatial_z_m  # Camera Z (forward) becomes robot X (forward)
            robot_frame_y = -spatial_x_m  # Camera X (right) becomes robot -Y (left)

            # Then rotate robot frame to world frame based on heading
            dx = robot_frame_x * math.cos(heading_rad) - robot_frame_y * math.sin(
                heading_rad
            )
            dy = robot_frame_x * math.sin(heading_rad) + robot_frame_y * math.cos(
                heading_rad
            )

            self.get_logger().info(
                f"Coordinate transformation: "
                f"camera=({spatial_x_m:.2f}, {spatial_y_m:.2f}, {spatial_z_m:.2f})m → "
                f"robot=({robot_frame_x:.2f}, {robot_frame_y:.2f})m → "
                f"world=({dx:.2f}, {dy:.2f})m with heading={current_heading:.2f}°"
            )
        else:
            # Fall back to angle-based calculation if spatial data is not available
            dx = distance_m * math.sin(global_angle_rad)
            dy = distance_m * math.cos(global_angle_rad)

            self.get_logger().debug(
                f"Using angle-based calculation: angle={global_angle:.2f}°, "
                f"distance={distance_m:.2f}m, dx={dx:.2f}m, dy={dy:.2f}m"
            )

        # Calculate obstacle position in map coordinates (meters)
        obstacle_x = robot_x + dx
        obstacle_y = robot_y + dy

        # Convert to grid coordinates
        grid_x = int((obstacle_x - self.map_origin_x) / self.map_resolution)
        grid_y = int((obstacle_y - self.map_origin_y) / self.map_resolution)

        # Calculate grid coordinates of robot for debugging
        robot_grid_x = int((robot_x - self.map_origin_x) / self.map_resolution)
        robot_grid_y = int((robot_y - self.map_origin_y) / self.map_resolution)

        # Calculate relative position in grid coordinates for debugging
        rel_grid_x = grid_x - robot_grid_x
        rel_grid_y = grid_y - robot_grid_y

        self.get_logger().debug(
            f"Robot grid: ({robot_grid_x}, {robot_grid_y}), "
            f"Obstacle grid: ({grid_x}, {grid_y}), "
            f"Relative grid: ({rel_grid_x}, {rel_grid_y})"
        )

        # Calculate obstacle size using height as diameter
        obstacle_diameter_m = height_m

        # Calculate obstacle radius in grid cells
        obstacle_radius = max(1, int(obstacle_diameter_m / (2 * self.map_resolution)))

        self.get_logger().info(
            f"Robot at ({robot_x:.2f}m, {robot_y:.2f}m), Obstacle at grid ({grid_x}, {grid_y}), "
            f"height {height_m:.2f}m, diameter {obstacle_diameter_m:.2f}m, radius {obstacle_radius} cells"
        )

        # Check if coordinates are within map bounds
        if (
            grid_x < 0
            or grid_x >= self.map_width
            or grid_y < 0
            or grid_y >= self.map_height
        ):
            self.get_logger().warn(
                f"Obstacle coordinates ({grid_x}, {grid_y}) outside map bounds"
            )
            return

        # Store this detection for visualization
        detection_info = {
            "x": obstacle_x,
            "y": obstacle_y,
            "grid_x": grid_x,
            "grid_y": grid_y,
            "radius": obstacle_radius,
            "height": height_m,
            "distance": distance_m,
            "angle": angle,
            "timestamp": self.get_clock().now().to_msg().sec,
        }

        # Add spatial data if available
        if has_spatial_data:
            detection_info.update(
                {
                    "spatial_x": spatial_x,
                    "spatial_y": spatial_y,
                    "spatial_z": spatial_z,
                    "confidence": confidence,
                }
            )

        # Add to recent detections list
        self.recent_detections.append(detection_info)

        # Keep only the most recent detections
        if len(self.recent_detections) > self.max_recent_detections:
            self.recent_detections.pop(0)

        # Update map with obstacle
        self.update_map_with_obstacle(grid_x, grid_y, obstacle_radius)

    def update_map_with_obstacle(
        self, grid_x: int, grid_y: int, obstacle_radius: int
    ) -> None:
        """Update map with an obstacle at specified grid coordinates.

        Args:
            grid_x: X coordinate in grid
            grid_y: Y coordinate in grid
            obstacle_radius: Radius of obstacle in grid cells
        """
        if self.map is None:
            self.get_logger().warn("No map data available")
            return

        # Create copy of map data
        map_data = list(self.map.data)
        cells_updated = 0

        # Update cells within obstacle radius
        for dx in range(-obstacle_radius, obstacle_radius + 1):
            for dy in range(-obstacle_radius, obstacle_radius + 1):
                # Calculate distance from center
                distance = math.sqrt(dx**2 + dy**2)

                # Only update cells within radius
                if distance <= obstacle_radius:
                    x = grid_x + dx
                    y = grid_y + dy

                    # Check if coordinates are within map bounds
                    if 0 <= x < self.map_width and 0 <= y < self.map_height:
                        # Calculate index in 1D array
                        index = y * self.map_width + x

                        # Update cell value if in bounds
                        if 0 <= index < len(map_data):
                            # Calculate occupancy based on distance from center
                            # Use a more gradual falloff to create more visible obstacles
                            distance_ratio = distance / obstacle_radius
                            if distance_ratio < 0.3:  # Core of the obstacle
                                occupancy = self.OBSTACLE_VALUE
                            else:
                                # Exponential falloff from center
                                occupancy = int(
                                    self.OBSTACLE_VALUE * math.exp(-2 * distance_ratio)
                                )

                            # Update with a weighted average to allow for obstacle reinforcement
                            current_value = map_data[index]
                            if current_value < 0:  # Unknown space
                                map_data[index] = occupancy
                                cells_updated += 1
                            elif current_value < occupancy:  # Lower occupancy
                                map_data[index] = occupancy
                                cells_updated += 1
                            elif (
                                current_value < 100 and occupancy > 0
                            ):  # Reinforce existing obstacle
                                # Weighted average with more weight to higher value
                                map_data[index] = min(
                                    100, int(current_value * 0.7 + occupancy * 0.3)
                                )
                                cells_updated += 1

        # Create new map message
        updated_map = OccupancyGrid()
        updated_map.header = self.map.header
        updated_map.header.stamp = self.get_clock().now().to_msg()

        # Ensure frame_id is set
        if not updated_map.header.frame_id:
            updated_map.header.frame_id = "world"

        updated_map.info = self.map.info
        updated_map.data = map_data

        # Update our stored map data
        self.map = updated_map

        # Publish updated map
        self.map_publisher.publish(updated_map)
        self.get_logger().info(
            f"Map updated with obstacle at ({grid_x}, {grid_y}), {cells_updated} cells updated"
        )

    def get_robot_map_position(self) -> Tuple[float, float]:
        """Get the robot's position in map coordinates.

        If GPS data is available, uses that. Otherwise, assumes the robot is in the center of the map.

        Returns:
            Tuple (x, y) of robot position in meters
        """
        # Check if we have valid GPS data
        if self.current_latitude is not None and self.current_longitude is not None:
            # Log that we don't have GPS conversion implemented
            self.get_logger().debug(
                "GPS to map conversion not implemented, using map center"
            )

            # Fallback to map center
            center_x = self.map_origin_x + (self.map_width * self.map_resolution) / 2
            center_y = self.map_origin_y + (self.map_height * self.map_resolution) / 2

            # Log the map center coordinates for debugging
            grid_center_x = int((center_x - self.map_origin_x) / self.map_resolution)
            grid_center_y = int((center_y - self.map_origin_y) / self.map_resolution)
            self.get_logger().debug(
                f"Map center in grid coordinates: ({grid_center_x}, {grid_center_y})"
            )

            return center_x, center_y
        else:
            # If no GPS data, assume robot is at center of map
            center_x = self.map_origin_x + (self.map_width * self.map_resolution) / 2
            center_y = self.map_origin_y + (self.map_height * self.map_resolution) / 2

            # Log the map center coordinates for debugging
            grid_center_x = int((center_x - self.map_origin_x) / self.map_resolution)
            grid_center_y = int((center_y - self.map_origin_y) / self.map_resolution)
            self.get_logger().debug(
                f"Map center in grid coordinates: ({grid_center_x}, {grid_center_y})"
            )
            self.get_logger().debug(
                f"No GPS data, assuming robot at map center ({center_x:.2f}, {center_y:.2f})"
            )

            return center_x, center_y

    def check_save_map(self) -> None:
        """Check if it's time to save a map image."""
        if not self.debug_save_maps or self.map is None:
            return

        current_time = time.time()
        time_since_last = current_time - self.last_save_time

        if time_since_last >= self.save_interval:
            self.get_logger().info(
                f"Time to save map image (interval: {self.save_interval}s)"
            )
            self.save_map_image()
            self.last_save_time = current_time

    def save_map_image(self) -> None:
        """Save the current map as an image."""
        if self.map is None:
            self.get_logger().warn("No map data available to save")
            return

        try:
            # Create timestamp for filename
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(self.save_dir, f"map_{timestamp}.png")

            # Convert map data to numpy array
            map_data = np.array(self.map.data).reshape(self.map_height, self.map_width)

            # Create a more detailed colormap for better visualization
            cmap = colors.LinearSegmentedColormap.from_list(
                "obstacle_map",
                [
                    (0.7, 0.7, 0.7),  # Light gray for unknown (-1)
                    (1.0, 1.0, 1.0),  # White for free space (0)
                    (1.0, 1.0, 0.0),  # Yellow for low occupancy
                    (1.0, 0.5, 0.0),  # Orange for medium occupancy
                    (1.0, 0.0, 0.0),
                ],  # Red for high occupancy
                N=102,
            )

            # Normalize the data for visualization
            # Convert -1 to 0, shift everything else up by 1
            vis_data = np.copy(map_data)
            vis_data = np.where(vis_data < 0, 0, vis_data + 1)

            # Create figure and plot
            plt.figure(figsize=(12, 12))
            plt.imshow(vis_data, cmap=cmap, vmin=0, vmax=101, origin="lower")

            # Add custom colorbar
            cbar = plt.colorbar(ticks=[0, 1, 26, 51, 76, 101])
            cbar.ax.set_yticklabels(["Unknown", "Free", "25%", "50%", "75%", "100%"])
            cbar.set_label("Occupancy")

            # Add grid
            plt.grid(which="both", color="lightgray", linestyle="-", alpha=0.5)

            # Add title with timestamp, map info, heading, and detection count
            heading_info = (
                f"Heading: {self.current_heading:.2f}°"
                if self.current_heading is not None
                else "Heading: N/A"
            )
            detection_count = (
                f"Detections: {len(self.recent_detections)}"
                if self.recent_detections
                else "Detections: 0"
            )
            plt.title(
                f"Map {timestamp}\n{self.map_width}x{self.map_height}, resolution: {self.map_resolution}m\n{heading_info} | {detection_count}"
            )

            # Add robot position if available
            robot_x, robot_y = self.get_robot_map_position()
            grid_x = int((robot_x - self.map_origin_x) / self.map_resolution)
            grid_y = int((robot_y - self.map_origin_y) / self.map_resolution)
            plt.plot(grid_x, grid_y, "bo", markersize=10, label="Robot")

            # Draw robot heading indicator
            if self.current_heading is not None:
                heading_rad = math.radians(self.current_heading)
                heading_length = 20  # Length of the heading indicator in grid cells
                heading_dx = heading_length * math.sin(heading_rad)
                heading_dy = heading_length * math.cos(heading_rad)
                plt.arrow(
                    grid_x,
                    grid_y,
                    heading_dx,
                    heading_dy,
                    head_width=5,
                    head_length=5,
                    fc="blue",
                    ec="blue",
                    label="Heading",
                )

            # Add detection markers for recent detections
            if self.recent_detections:
                # Get the number of closest detections to display
                max_displayed = self.get_parameter("max_displayed_detections").value

                # Get the closest detections
                sorted_detections = self.get_closest_detections(max_displayed)

                # Plot recent detections with different colors based on age
                for i, detection in enumerate(self.recent_detections):
                    det_grid_x = int(
                        (detection["x"] - self.map_origin_x) / self.map_resolution
                    )
                    det_grid_y = int(
                        (detection["y"] - self.map_origin_y) / self.map_resolution
                    )

                    # Use different marker styles based on age
                    alpha = 0.5 + 0.5 * (
                        i / len(self.recent_detections)
                    )  # Newer detections are more opaque

                    # Plot detection point
                    plt.plot(det_grid_x, det_grid_y, "r.", markersize=4, alpha=alpha)

                    # Only draw circle for the latest detection
                    if i == len(self.recent_detections) - 1 and "radius" in detection:
                        circle = plt.Circle(
                            (det_grid_x, det_grid_y),
                            detection["radius"],
                            fill=False,
                            color="red",
                            alpha=0.7,
                        )
                        plt.gca().add_patch(circle)

                # Add detection info to the plot
                info_text = f"Total Detections: {len(self.recent_detections)}\n\n"
                info_text += f"{max_displayed} Closest Detections:\n"

                # Add info for the closest detections (or fewer if there aren't enough)
                for i, detection in enumerate(sorted_detections[:max_displayed]):
                    det_grid_x = int(
                        (detection["x"] - self.map_origin_x) / self.map_resolution
                    )
                    det_grid_y = int(
                        (detection["y"] - self.map_origin_y) / self.map_resolution
                    )
                    info_text += f"{i+1}. Grid: ({det_grid_x}, {det_grid_y}), "
                    info_text += f"Dist: {detection['distance_to_robot']:.2f}m"
                    if i < len(sorted_detections[:max_displayed]) - 1:
                        info_text += "\n"

                # Add text box with detection info
                plt.figtext(
                    0.02, 0.02, info_text, bbox=dict(facecolor="white", alpha=0.7)
                )

            # Add grid coordinates
            plt.grid(True)
            plt.xlabel("X (grid cells)")
            plt.ylabel("Y (grid cells)")

            # Add legend
            plt.legend(loc="upper left")

            # Save figure
            plt.savefig(filename, dpi=150, bbox_inches="tight")
            plt.close()

            self.get_logger().info(f"Saved map image to {filename}")

            # Cleanup old maps
            self.cleanup_old_maps()

        except Exception as e:
            self.get_logger().error(f"Error saving map image: {str(e)}")

    def cleanup_old_maps(self) -> None:
        """Keep only the most recent map images."""
        try:
            # Get list of map images sorted by modification time (newest first)
            maps = sorted(
                [
                    os.path.join(self.save_dir, f)
                    for f in os.listdir(self.save_dir)
                    if f.endswith(".png") and f.startswith("map_")
                ],
                key=os.path.getmtime,
                reverse=True,
            )

            # Remove older maps
            for map_file in maps[self.max_maps :]:
                os.remove(map_file)
                self.get_logger().info(f"Removed old map image: {map_file}")

        except Exception as e:
            self.get_logger().error(f"Error cleaning up map images: {e}")


def main(args=None):
    """Entry point for the obstacle mapper node."""
    rclpy.init(args=args)
    node = ObstacleMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
