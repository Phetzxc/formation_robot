# #!/usr/bin/env python3
# import cv2
# import numpy as np
# import os
# import time
# import glob

# class ImageRectifier:
#     def __init__(self):
#         self.image_paths = {
#             'front': 'front_undistorted_*.png',
#             'back': 'back_undistorted_*.png',
#             'left': 'left_undistorted_*.png',
#             'right': 'right_undistorted_*.png'
#         }
#         self.output_folder = "corrected_images"
#         os.makedirs(self.output_folder, exist_ok=True)

#         # Load latest images
#         self.latest_images = self.load_latest_images()

#         # Check if all images are loaded
#         if any(img is None for img in self.latest_images.values()):
#             print("Error: Some images could not be loaded.")
#             return

#         # Camera Intrinsic Parameters (from URDF)
#         self.camera_matrix = np.array([[800, 0, 320],
#                                        [0, 800, 240],
#                                        [0,   0,   1]], dtype=np.float32)
#         self.dist_coeffs = np.array([-0.2, 0.1, 0, 0, 0], dtype=np.float32)  # Example fisheye distortion

#         # Compute transformation matrices
#         self.homographies = self.compute_homographies()

#         # Apply transformations and save corrected images
#         self.process_and_save_images()

#     def load_latest_images(self):
#         """Loads the latest saved images from each camera."""
#         latest_images = {}
#         for cam, pattern in self.image_paths.items():
#             files = sorted(glob.glob(pattern), key=os.path.getctime, reverse=True)
#             if files:
#                 latest_images[cam] = cv2.imread(files[0])
#                 print(f"Loaded {files[0]} for {cam}.")
#             else:
#                 print(f"Warning: No file found for {cam}.")
#                 latest_images[cam] = None
#         return latest_images

#     def compute_homographies(self):
#         """Computes homography matrices for perspective correction."""
#         h, w = 480, 640  # Image size

#         # Define source points (distorted images from cameras)
#         src_points = np.array([
#             [50, 50], [w-50, 50],  # Top-left, Top-right
#             [50, h-50], [w-50, h-50]  # Bottom-left, Bottom-right
#         ], dtype=np.float32)

#         # Define a standard projection (aligned view)
#         dst_points = np.array([
#             [0, 0], [w, 0],
#             [0, h], [w, h]
#         ], dtype=np.float32)

#         homographies = {}
#         for cam in ['front', 'back', 'left', 'right']:
#             homographies[cam] = cv2.getPerspectiveTransform(src_points, dst_points)

#         return homographies

#     def process_and_save_images(self):
#         """Applies transformations and saves corrected images."""
#         timestamp = int(time.time())

#         for cam, img in self.latest_images.items():
#             if img is None:
#                 continue

#             # Undistort image (fisheye correction)
#             map1, map2 = cv2.initUndistortRectifyMap(
#                 self.camera_matrix, self.dist_coeffs, None, self.camera_matrix, (640, 480), cv2.CV_16SC2)
#             undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

#             # Apply perspective transformation
#             transformed_img = cv2.warpPerspective(undistorted_img, self.homographies[cam], (640, 480))

#             # Save the corrected image
#             corrected_filename = os.path.join(self.output_folder, f"{cam}_corrected_{timestamp}.png")
#             cv2.imwrite(corrected_filename, transformed_img)
#             print(f"Saved {corrected_filename}")

# if __name__ == "__main__":
#     ImageRectifier()


# #!/usr/bin/env python3
# import cv2
# import numpy as np
# import os
# import time
# import glob

# class ImageRectifier:
#     def __init__(self):
#         self.image_paths = {
#             'front': 'front_undistorted_*.png',
#             'back': 'back_undistorted_*.png',
#             'left': 'left_undistorted_*.png',
#             'right': 'right_undistorted_*.png'
#         }
#         self.output_folder = "corrected_images"
#         os.makedirs(self.output_folder, exist_ok=True)

#         # Load latest images
#         self.latest_images = self.load_latest_images()

#         if any(img is None for img in self.latest_images.values()):
#             print("Error: Some images could not be loaded.")
#             return

#         # Camera Intrinsic Parameters (จาก URDF)
#         self.camera_matrix = np.array([
#             [800, 0, 320],  
#             [0, 800, 240],  
#             [0, 0, 1]       
#         ], dtype=np.float32)

#         # Distortion Coefficients (ปรับให้เหมาะสมกับเลนส์ 160°)
#         self.dist_coeffs = np.array([-0.27, 0.8, 0.0002, 0.0002, -0.12], dtype=np.float32)


#         # คำนวณ new camera matrix สำหรับ undistortion
#         self.new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
#             self.camera_matrix, self.dist_coeffs, (640, 480), 1, (640, 480))

#         # Compute transformation matrices
#         self.homographies = self.compute_homographies()

#         # Apply transformations and save corrected images
#         self.process_and_save_images()

#     def load_latest_images(self):
#         """โหลดภาพล่าสุดจากแต่ละกล้อง"""
#         latest_images = {}
#         for cam, pattern in self.image_paths.items():
#             files = sorted(glob.glob(pattern), key=os.path.getctime, reverse=True)
#             if files:
#                 latest_images[cam] = cv2.imread(files[0])
#                 print(f"Loaded {files[0]} for {cam}.")
#             else:
#                 print(f"Warning: No file found for {cam}.")
#                 latest_images[cam] = None
#         return latest_images

#     def compute_homographies(self):
#         """คำนวณ homography สำหรับ perspective correction"""
#         h, w = 480, 640  

#         # ปรับค่าตำแหน่งจุดให้ใกล้เคียงกับมุมมองที่ถูกต้อง
#         src_points = np.array([
#             [50, 40], [w-50, 40],  
#             [50, h-40], [w-50, h-40]  
#         ], dtype=np.float32)

#         dst_points = np.array([
#             [0, 0], [w, 0],
#             [0, h], [w, h]
#         ], dtype=np.float32)

#         homographies = {}
#         for cam in ['front', 'back', 'left', 'right']:
#             homographies[cam] = cv2.getPerspectiveTransform(src_points, dst_points)

#         return homographies

#     def process_and_save_images(self):
#         """แปลงภาพและบันทึกภาพที่ปรับแล้ว"""
#         timestamp = int(time.time())

#         for cam, img in self.latest_images.items():
#             if img is None:
#                 continue

#             # **✅ แก้ไขการบิดเบี้ยวของเลนส์**
#             map1, map2 = cv2.initUndistortRectifyMap(
#                 self.camera_matrix, self.dist_coeffs, None, self.new_camera_matrix, (640, 480), cv2.CV_16SC2)
#             undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

#             # **✅ ใช้ Homography Transformation**
#             transformed_img = cv2.warpPerspective(undistorted_img, self.homographies[cam], (640, 480))

#             # **✅ Save ภาพที่แก้ไขแล้ว**
#             corrected_filename = os.path.join(self.output_folder, f"{cam}_corrected_{timestamp}.png")
#             cv2.imwrite(corrected_filename, transformed_img)
#             print(f"Saved {corrected_filename}")

# if __name__ == "__main__":
#     ImageRectifier()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
import glob
import time

class ImageReverter(Node):
    def __init__(self):
        super().__init__('image_reverter')
        self.image_folder = "undistorted_images"
        self.output_folder = "reverted_images"
        
        # Create directories if they do not exist
        os.makedirs(self.image_folder, exist_ok=True)
        os.makedirs(self.output_folder, exist_ok=True)

        # Camera Intrinsic Parameters
        self.camera_matrix = np.array([
            [320, 0, 320],
            [0, 320, 240],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # Distortion Coefficients (original fisheye distortion)
        self.dist_coeffs = np.array([-0.3, 0.1, 0.0, 0.0], dtype=np.float32)

        # Process the latest images
        self.revert_images()

    def revert_images(self):
        latest_images = self.load_latest_images()
        timestamp = int(time.time())

        for cam, img in latest_images.items():
            if img is None:
                continue

            reverted_img = self.apply_fisheye_distortion(img)
            filename = os.path.join(self.output_folder, f"{cam}_reverted.png")
            cv2.imwrite(filename, reverted_img)
            self.get_logger().info(f"Saved {filename}")

    def load_latest_images(self):
        latest_images = {}
        for cam in ['front', 'back', 'left', 'right']:
            pattern = os.path.join(self.image_folder, f"{cam}_undistorted.png")
            files = sorted(glob.glob(pattern), key=os.path.getctime, reverse=True)
            if files:
                latest_images[cam] = cv2.imread(files[0])
                self.get_logger().info(f"Loaded {files[0]} for {cam}.")
            else:
                self.get_logger().warn(f"No file found for {cam}.")
                latest_images[cam] = None
        return latest_images

    def apply_fisheye_distortion(self, img):
        h, w = img.shape[:2]
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
        
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            self.camera_matrix, -self.dist_coeffs, np.eye(3), new_camera_matrix, (w, h), cv2.CV_16SC2)
        reverted = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return reverted


def main(args=None):
    rclpy.init(args=args)
    node = ImageReverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
