import rclpy
from rclpy.node import Node
from .camera_localizer import CameraLocalizer, CameraLocalizerDetection
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from threading import Thread
import imageio
import numpy as np
import cv2
from cv_bridge import CvBridge
import os
from glob import glob
import time
from ament_index_python.packages import get_package_share_directory

class CameraLocalizerNode(Node):
    def __init__(self):
        super().__init__('camera_localizer_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'detections', 10)
        self.image_publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.bridge = CvBridge()

        # Initialize CameraLocalizer
        package_share_dir = get_package_share_directory('obstacle_detection')
        models_dir = os.path.join(package_share_dir, 'models')
        
        # Find all .blob files in the models directory
        blob_files = glob(os.path.join(models_dir, '**/*.blob'), recursive=True)
        
        if not blob_files:
            self.get_logger().error(f'No .blob files found in {models_dir}')
            raise FileNotFoundError(f'No blob model files found in {models_dir}')
            
        # Use the first blob file found
        model_path = blob_files[0]
        self.get_logger().info(f'Using model: {model_path}')
            
        self.camera_localizer = CameraLocalizer(model_path=model_path)
        
        # Start the camera in a separate thread
        self.camera_thread = Thread(target=self.run_camera_localizer)
        self.camera_thread.start()

        self.debug_save_gifs = True  # Set to False to disable GIF saving
        # Initialize GIF-related variables only if debug is enabled
        if self.debug_save_gifs:
            self.frame_buffer = []
            self.max_frames = 30  # Number of frames to store for GIF
            self.save_dir = '/home/index-finger/Source/SJSU_Robotics/urc_intelsys_2024/camera_gifs'
            os.makedirs(self.save_dir, exist_ok=True)
            self.last_save_time = time.time()
            self.save_interval = 5.0  # Save GIF every 5 seconds
            self.max_gifs = 10  # Maximum number of GIFs to keep

        # Create timer to call timer_callback periodically
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz timer

    def cleanup_old_gifs(self):
        """Keep only the most recent GIFs"""
        try:
            # Get list of GIFs sorted by modification time (newest first)
            gifs = sorted(
                [os.path.join(self.save_dir, f) for f in os.listdir(self.save_dir) if f.endswith('.gif')],
                key=os.path.getmtime,
                reverse=True
            )
            
            # Remove older GIFs
            for gif in gifs[self.max_gifs:]:
                os.remove(gif)
                self.get_logger().info(f'Removed old GIF: {gif}')
        except Exception as e:
            self.get_logger().error(f'Error cleaning up GIFs: {e}')

    def run_camera_localizer(self):
        self.camera_localizer.start(blocking=False)



    def timer_callback(self):
        detections = self.camera_localizer.getDetections()
        frame = self.camera_localizer.get_current_frame()

        if frame is not None:
            height, width = frame.shape[:2]
            color = (255, 255, 255)
            fontColor = (0, 0, 255)

            for detection in detections:
                # Denormalize bounding box
                x1 = int(detection.xmin * width)
                x2 = int(detection.xmax * width)
                y1 = int(detection.ymin * height)
                y2 = int(detection.ymax * height)

                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

                # Add text information
                increment = 15
                start = y1 + 20
                texts = [
                    "rock",
                    f"{detection.confidence * 100:.2f}",
                    f"X: {int(detection.spatial_x)} mm",
                    f"Y: {int(detection.spatial_y)} mm",
                    f"Z: {int(detection.spatial_z)} mm",
                    f"H*: {int(detection.angle)} deg",
                    f"HE: {int(detection.height)} mm",
                ]

                for idx, text in enumerate(texts):
                    cv2.putText(
                        frame,
                        text,
                        (x1 + 10, start + increment * idx),
                        cv2.FONT_HERSHEY_TRIPLEX,
                        0.5,
                        fontColor,
                    )

            # GIF saving logic only runs if debug flag is enabled
            # GIF saving logic only runs if debug flag is enabled
            if self.debug_save_gifs:
                self.get_logger().info(f'Debug GIF saving is enabled')
                self.frame_buffer.append(cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB))
                                
                # Keep only the last max_frames frames
                if len(self.frame_buffer) > self.max_frames:
                    self.frame_buffer.pop(0)

                # Save GIF every save_interval seconds
                current_time = time.time()
                time_since_last = current_time - self.last_save_time
                self.get_logger().info(f'Time since last save: {time_since_last:.2f}s')
                
                if time_since_last >= self.save_interval and len(self.frame_buffer) > 0:
                    self.get_logger().info(f'Attempting to save GIF. Buffer size: {len(self.frame_buffer)}')
                    timestamp = time.strftime("%Y%m%d-%H%M%S")
                    filename = os.path.join(self.save_dir, f'camera_sequence_{timestamp}.gif')
                    
                    try:
                        # Save GIF with original framerate
                        self.get_logger().info(f'Saving GIF to: {filename}')
                        imageio.mimsave(filename, self.frame_buffer, duration=0.033, loop=0)
                        self.get_logger().info(f'Successfully saved GIF to {filename}')
                        
                        # Cleanup old GIFs
                        self.cleanup_old_gifs()
                        
                        # Clear buffer and reset timer
                        self.frame_buffer = []
                        self.last_save_time = current_time
                    except Exception as e:
                        self.get_logger().error(f'Error saving GIF: {str(e)}')
                        self.get_logger().error(f'Save directory exists: {os.path.exists(self.save_dir)}')
                        self.get_logger().error(f'Save directory permissions: {oct(os.stat(self.save_dir).st_mode)[-3:]}')



            # Convert to ROS Image message and publish
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_publisher_.publish(image_msg)

            # Publish detection data
            for detection in detections:
                msg = Float32MultiArray()
                msg.data = [detection.angle, detection.height, detection.distance]
                self.publisher_.publish(msg)

    def destroy_node(self):
        self.camera_localizer.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()