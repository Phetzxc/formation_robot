#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.image_paths = {
            'front': '/camera_front_sensor/image_raw',
            'back': '/camera_back_sensor/image_raw',
            'left': '/camera_left_sensor/image_raw',
            'right': '/camera_right_sensor/image_raw'
        }
        self.output_folder = "saved_images"
        os.makedirs(self.output_folder, exist_ok=True)

        # Subscribe to the four camera topics
        for cam, topic in self.image_paths.items():
            self.create_subscription(Image, topic, lambda msg, c=cam: self.image_callback(msg, c), 10)
        
        self.get_logger().info("Image Saver node started.")

    def image_callback(self, msg, cam_name):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            timestamp = int(time.time())
            filename = os.path.join(self.output_folder, f"{cam_name}_raw.png")
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved {filename}")
        except CvBridgeError as e:
            self.get_logger().error(f"{cam_name} conversion error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
