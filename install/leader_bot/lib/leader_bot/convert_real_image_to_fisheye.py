import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
import os

class PanoramaStitcher(Node):
    def __init__(self):
        super().__init__('panorama_stitcher')
        self.bridge = CvBridge()
        self.latest_images = {'front': None, 'back': None, 'left': None, 'right': None}

        # Subscribe to the four camera topics
        self.create_subscription(Image, '/camera_front_sensor/image_raw', self.front_callback, 10)
        self.create_subscription(Image, '/camera_back_sensor/image_raw', self.back_callback, 10)
        self.create_subscription(Image, '/camera_left_sensor/image_raw', self.left_callback, 10)
        self.create_subscription(Image, '/camera_right_sensor/image_raw', self.right_callback, 10)

        # Process images every 15 seconds
        self.create_timer(15.0, self.timer_callback)

        # Set up folder for saving undistorted images
        self.output_folder = "undistorted_images"
        os.makedirs(self.output_folder, exist_ok=True)

        # Fisheye camera parameters (adjust these)
        self.DIM = (640, 480)
        self.K = np.array([[320, 0, 320],
                           [0, 320, 240],
                           [0,   0,   1]], dtype=np.float32)
        self.D = np.array([-0.3, 0.1, 0.0, 0.0], dtype=np.float32)

        self.get_logger().info("Panorama Stitcher node started.")

    def front_callback(self, msg):
        self.latest_images['front'] = self.convert_and_undistort(msg, 'front')

    def back_callback(self, msg):
        self.latest_images['back'] = self.convert_and_undistort(msg, 'back')

    def left_callback(self, msg):
        self.latest_images['left'] = self.convert_and_undistort(msg, 'left')

    def right_callback(self, msg):
        self.latest_images['right'] = self.convert_and_undistort(msg, 'right')

    def convert_and_undistort(self, msg, cam_name):
        """Convert ROS image message to OpenCV format and undistort"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            undistorted = self.undistort_image(cv_image)
            return self.apply_fisheye_effect(undistorted)
        except CvBridgeError as e:
            self.get_logger().error(f"{cam_name} conversion error: {e}")
            return None

    def undistort_image(self, img):
        """Undistort the image using fisheye correction"""
        w, h = self.DIM
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), self.K, (w, h), cv2.CV_16SC2)
        return cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    def apply_fisheye_effect(self, img):
        """Applies artificial fisheye effect"""
        h, w = img.shape[:2]
        K = np.array([[w, 0, w / 2],
                      [0, h, h / 2],
                      [0, 0, 1]], dtype=np.float32)
        D = np.array([0.3, -0.2, 0.0, 0.0], dtype=np.float32)

        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (w, h), cv2.CV_16SC2)
        return cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    def timer_callback(self):
        """Process images and save results"""
        if any(img is None for img in self.latest_images.values()):
            self.get_logger().warn("Not all images received yet.")
            return

        timestamp = int(time.time())

        # Save undistorted images to folder
        for cam_name, img in self.latest_images.items():
            filename = os.path.join(self.output_folder, f"{cam_name}_undistorted.png")
            cv2.imwrite(filename, img)
            self.get_logger().info(f"Saved {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = PanoramaStitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
