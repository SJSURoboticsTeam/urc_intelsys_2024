import rclpy
from rclpy.node import Node
from .camera_localizer import CameraLocalizer, CameraLocalizerDetection
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from threading import Thread
import numpy as np
import cv2
from cv_bridge import CvBridge
import os
from glob import glob
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

        # Create a timer to periodically publish detections
        self.timer = self.create_timer(0.1, self.timer_callback)

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