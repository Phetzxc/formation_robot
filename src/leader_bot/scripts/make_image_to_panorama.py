# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import cv2
# import numpy as np
# import os
# import time
# import glob

# class PanoramaGenerator(Node):
#     def __init__(self):
#         super().__init__('panorama_generator')

#         self.image_paths = {
#             'front': 'corrected_images/front_corrected_*.png',
#             'back': 'corrected_images/back_corrected_*.png',
#             'left': 'corrected_images/left_corrected_*.png',
#             'right': 'corrected_images/right_corrected_*.png'
#         }
#         self.output_folder = "panorama_images"
#         os.makedirs(self.output_folder, exist_ok=True)

#         # Load latest images
#         self.latest_images = self.load_latest_images()

#         # Check if all images are loaded
#         if any(img is None for img in self.latest_images.values()):
#             self.get_logger().error("Some images could not be loaded.")
#             return

#         # Process images
#         self.process_and_stitch_images()

#     def load_latest_images(self):
#         """Loads the latest saved images from each camera."""
#         latest_images = {}
#         for cam, pattern in self.image_paths.items():
#             files = sorted(glob.glob(pattern), key=os.path.getctime, reverse=True)
#             if files:
#                 latest_images[cam] = cv2.imread(files[0])
#                 self.get_logger().info(f"Loaded {files[0]} for {cam}.")
#             else:
#                 self.get_logger().warn(f"No file found for {cam}.")
#                 latest_images[cam] = None
#         return latest_images

#     def crop_overlap(self, img, side):
#         """ตัดส่วนที่ซ้อนทับกันออก"""
#         h, w, _ = img.shape
#         crop_size = int(w * 0.15)  # ตัดขอบที่ซ้อน

#         if side == 'front':
#             return img[:, crop_size:-crop_size]  # ตัดซ้าย-ขวา
#         elif side == 'left':
#             return img[:, :-crop_size]  # ตัดขอบขวา
#         elif side == 'right':
#             return img[:, crop_size:]  # ตัดขอบซ้าย
#         elif side == 'back':
#             half_w = w // 2
#             left_part = img[:, :half_w]  # ซ้าย
#             right_part = img[:, half_w:]  # ขวา
#             return left_part, right_part
#         return img

#     def process_and_stitch_images(self):
#         """ตัดขอบที่ซ้อน และต่อภาพเป็น Panorama"""
#         timestamp = int(time.time())

#         # ตัดส่วนซ้อนทับ
#         front = self.crop_overlap(self.latest_images['front'], 'front')
#         left = self.crop_overlap(self.latest_images['left'], 'left')
#         right = self.crop_overlap(self.latest_images['right'], 'right')
#         back_left, back_right = self.crop_overlap(self.latest_images['back'], 'back')

#         # ต่อภาพ
#         panorama = cv2.hconcat([back_left, left, front, right, back_right])

#         # บันทึก
#         panorama_filename = os.path.join(self.output_folder, f"panorama_{timestamp}.png")
#         cv2.imwrite(panorama_filename, panorama)
#         self.get_logger().info(f"Saved panorama image as {panorama_filename}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = PanoramaGenerator()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()
####################################################################################################################################
# #!/usr/bin/env python3
# import cv2
# import numpy as np
# import os
# import time
# import glob

# class PanoramaGenerator:
#     def __init__(self):
#         self.image_paths = {
#             'front': 'corrected_images/front_corrected_*.png',
#             'back': 'corrected_images/back_corrected_*.png',
#             'left': 'corrected_images/left_corrected_*.png',
#             'right': 'corrected_images/right_corrected_*.png'
#         }
#         self.output_folder = "panorama_images"
#         os.makedirs(self.output_folder, exist_ok=True)

#         # Load latest images
#         self.latest_images = self.load_latest_images()

#         # Check if all images are loaded
#         if any(img is None for img in self.latest_images.values()):
#             print("Error: Some images could not be loaded.")
#             return

#         # Resize and Zoom-In Images
#         self.resize_and_zoom_images()

#         # Process and stitch images
#         self.process_and_stitch_images()

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

#     def resize_and_zoom_images(self):
#         """Ensure all images are the same size and adjust left, right, and back to match front."""
#         target_size = (640, 480)  # Standard size

#         # Resize `front` image as reference
#         self.latest_images['front'] = cv2.resize(self.latest_images['front'], target_size)

#         # Apply zoom-in transformation to side and back images
#         zoom_factor = 1.2  # Adjust for better alignment
#         for cam in ['left', 'right', 'back']:
#             img = self.latest_images[cam]
#             if img is not None:
#                 h, w = img.shape[:2]
#                 new_h, new_w = int(h * zoom_factor), int(w * zoom_factor)
#                 resized = cv2.resize(img, (new_w, new_h))

#                 # Crop center to maintain same size
#                 x_offset = (new_w - w) // 2
#                 y_offset = (new_h - h) // 2
#                 cropped = resized[y_offset:y_offset+h, x_offset:x_offset+w]

#                 self.latest_images[cam] = cropped
#                 print(f"Zoomed-in {cam} and cropped to match front.")

#     def process_and_stitch_images(self):
#         """Cuts overlapping parts and stitches images into a panorama."""
#         timestamp = int(time.time())
        
#         # Get images
#         left = self.latest_images['left']
#         front = self.latest_images['front']
#         right = self.latest_images['right']
#         back = self.latest_images['back']

#         # Define overlap width (amount to be cut from each side)
#         overlap = 80  # Adjust this value as needed

#         # Crop overlapping areas
#         left = left[:, :-overlap]  # Remove right part
#         front = front[:, overlap:-overlap]  # Remove both sides
#         right = right[:, overlap:]  # Remove left part

#         # Split back image in half and use both halves
#         back_left = back[:, :back.shape[1]//2]
#         back_right = back[:, back.shape[1]//2:]

#         # Concatenate images in order: back-left, left, front, right, back-right
#         panorama = np.hstack((back_left, left, front, right, back_right))

#         # Save the panorama image
#         panorama_filename = os.path.join(self.output_folder, f"panorama_{timestamp}.png")
#         cv2.imwrite(panorama_filename, panorama)
#         print(f"Saved panorama image as {panorama_filename}")

# if __name__ == "__main__":
#     PanoramaGenerator()

####################################################################################################################################
# #!/usr/bin/env python3
# import cv2
# import numpy as np
# import os
# import time
# import glob

# class ZoomPanoramaGenerator:
#     def __init__(self):
#         self.image_paths = {
#             'front': 'corrected_images/front_corrected_*.png',
#             'back':  'corrected_images/back_corrected_*.png',
#             'left':  'corrected_images/left_corrected_*.png',
#             'right': 'corrected_images/right_corrected_*.png'
#         }
#         self.output_folder = "panorama_images"
#         os.makedirs(self.output_folder, exist_ok=True)

#         # โหลดภาพ
#         self.latest_images = self.load_latest_images()

#         # ถ้าโหลดไม่ครบ ก็หยุด
#         if any(img is None for img in self.latest_images.values()):
#             print("Error: Some images could not be loaded.")
#             return

#         # front ใช้เป็น reference
#         self.target_size = (640, 480)
#         self.latest_images['front'] = cv2.resize(self.latest_images['front'], self.target_size)

#         # ปรับ scale ให้ match
#         self.adjust_scale_side('left', 'front')
#         self.adjust_scale_side('right', 'front')

#         # แบ่ง back ออกเป็น back_left, back_right และปรับ scale
#         self.split_and_adjust_back()

#         # สร้าง panorama โดยไม่มีขอบดำ
#         self.create_panorama()

#     def load_latest_images(self):
#         """โหลดภาพล่าสุดจากโฟลเดอร์ corrected_images"""
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

#     def adjust_scale_side(self, side_cam, ref_cam):
#         """
#         ปรับ scale ภาพ side_cam (เช่น left/right)
#         ให้ match สี overlap กับ ref_cam (front)
#         """
#         if self.latest_images[side_cam] is None: 
#             return

#         base_img = cv2.resize(self.latest_images[side_cam], self.target_size)

#         best_scale = 1.0
#         best_score = 999999999.0
#         scale_candidates = np.linspace(0.9, 1.1, 21)

#         for s in scale_candidates:
#             zoomed = self.zoom_image(base_img, s)
#             score = self.compare_overlap(zoomed, self.latest_images[ref_cam], side_cam, ref_cam)
#             if score < best_score:
#                 best_score = score
#                 best_scale = s

#         final_img = self.zoom_image(base_img, best_scale, remove_black=True)
#         self.latest_images[side_cam] = final_img
#         print(f"{side_cam} best_scale={best_scale:.2f}, score={best_score:.2f}")

#     def split_and_adjust_back(self):
#         """
#         แบ่งภาพ back เป็น 2 ซีก: back_left, back_right
#         แล้วปรับ scale โดยอิง left, right
#         """
#         if self.latest_images['back'] is None:
#             return

#         back_img = cv2.resize(self.latest_images['back'], self.target_size)
#         w = back_img.shape[1]
#         half_w = w // 2

#         back_left_img = back_img[:, :half_w]
#         back_right_img = back_img[:, half_w:]

#         best_scale_l = 1.0
#         best_score_l = 999999999.0
#         scale_candidates = np.linspace(0.9, 1.1, 21)

#         for s in scale_candidates:
#             zoomed = self.zoom_image(back_left_img, s)
#             score = self.compare_overlap(zoomed, self.latest_images['left'], 'back_left', 'left')
#             if score < best_score_l:
#                 best_score_l = score
#                 best_scale_l = s

#         final_bl = self.zoom_image(back_left_img, best_scale_l, remove_black=True)
#         print(f"back_left best_scale={best_scale_l:.2f}, score={best_score_l:.2f}")

#         best_scale_r = 1.0
#         best_score_r = 999999999.0
#         for s in scale_candidates:
#             zoomed = self.zoom_image(back_right_img, s)
#             score = self.compare_overlap(zoomed, self.latest_images['right'], 'back_right', 'right')
#             if score < best_score_r:
#                 best_score_r = score
#                 best_scale_r = s

#         final_br = self.zoom_image(back_right_img, best_scale_r, remove_black=True)
#         print(f"back_right best_scale={best_scale_r:.2f}, score={best_score_r:.2f}")

#         self.latest_images['back'] = np.hstack((final_bl, final_br))

#     def zoom_image(self, img, scale, remove_black=False):
#         """
#         zoom in/out ด้วย factor = scale แล้วครอปขอบดำออกถ้าต้องการ
#         """
#         h, w = img.shape[:2]
#         new_w = int(w * scale)
#         new_h = int(h * scale)
#         zoomed = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_CUBIC)

#         if remove_black:
#             mask = cv2.cvtColor(zoomed, cv2.COLOR_BGR2GRAY)
#             _, thresh = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)
#             contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#             if contours:
#                 x, y, w, h = cv2.boundingRect(contours[0])
#                 zoomed = zoomed[y:y+h, x:x+w]

#         zoomed = cv2.resize(zoomed, (640, 480), interpolation=cv2.INTER_CUBIC)
#         return zoomed

#     def compare_overlap(self, imgA, imgB, camA, camB):
#         """คำนวณคะแนนความต่างสีในบริเวณ overlap"""
#         overlap_size = 50

#         if (camA == 'left' and camB == 'front'):
#             partA = imgA[:, -overlap_size:]
#             partB = imgB[:, :overlap_size]
#         elif (camA == 'right' and camB == 'front'):
#             partA = imgA[:, :overlap_size]
#             partB = imgB[:, -overlap_size:]
#         elif (camA == 'back_left' and camB == 'left'):
#             partA = imgA[:, -overlap_size:]
#             partB = imgB[:, :overlap_size]
#         elif (camA == 'back_right' and camB == 'right'):
#             partA = imgA[:, :overlap_size]
#             partB = imgB[:, -overlap_size:]
#         else:
#             partA = imgA[:, :overlap_size]
#             partB = imgB[:, :overlap_size]

#         partA = cv2.resize(partA, (overlap_size, 480))
#         partB = cv2.resize(partB, (overlap_size, 480))

#         diff = cv2.absdiff(partA, partB)
#         return float(np.mean(diff))

#     def create_panorama(self):
#         """ตัด overlap และ stitch ภาพ"""
#         timestamp = int(time.time())

#         left = self.latest_images['left']
#         front = self.latest_images['front']
#         right = self.latest_images['right']
#         back = self.latest_images['back']

#         if any(x is None for x in [left, front, right, back]):
#             print("Some images are missing after scale adjustment.")
#             return

#         overlap = 50
#         w_b = back.shape[1]
#         half_w = w_b // 2
#         back_left = back[:, :half_w]
#         back_right = back[:, half_w:]

#         back_left = back_left[:, :-overlap]
#         left = left[:, :-overlap]
#         front = front[:, overlap:-overlap]
#         right = right[:, overlap:]
#         back_right = back_right[:, overlap:]

#         panorama = np.hstack((back_left, left, front, right, back_right))

#         pano_name = f"panorama_{timestamp}.png"
#         pano_path = os.path.join(self.output_folder, pano_name)
#         cv2.imwrite(pano_path, panorama)
#         print(f"Saved final panorama => {pano_path}")

# if __name__ == "__main__":
#     ZoomPanoramaGenerator()
####################################################################################################################################
# #!/usr/bin/env python3
# import cv2
# import numpy as np
# import os
# import time
# import glob

# class ZoomPanoramaGenerator:
#     def __init__(self):
#         self.image_paths = {
#             'front': 'corrected_images/front_corrected_*.png',
#             'back':  'corrected_images/back_corrected_*.png',
#             'left':  'corrected_images/left_corrected_*.png',
#             'right': 'corrected_images/right_corrected_*.png'
#         }
#         self.output_folder = "panorama_images"
#         os.makedirs(self.output_folder, exist_ok=True)

#         # โหลดภาพ
#         self.latest_images = self.load_latest_images()

#         # ถ้าโหลดไม่ครบ ก็หยุด
#         if any(img is None for img in self.latest_images.values()):
#             print("Error: Some images could not be loaded.")
#             return

#         # front ใช้เป็น reference
#         self.target_size = (640, 480)
#         self.latest_images['front'] = cv2.resize(self.latest_images['front'], self.target_size)

#         # ปรับ scale ให้ match
#         self.adjust_scale_side('left', 'front')
#         self.adjust_scale_side('right', 'front')

#         # แบ่ง back ออกเป็น back_left, back_right และปรับ scale
#         self.split_and_adjust_back()

#         # สร้าง panorama โดยไม่มีขอบดำ + ตัดขอบโค้ง
#         self.create_panorama()

#     def load_latest_images(self):
#         """โหลดภาพล่าสุดจากโฟลเดอร์ corrected_images"""
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

#     def adjust_scale_side(self, side_cam, ref_cam):
#         """
#         ปรับ scale ภาพ side_cam (เช่น left/right)
#         ให้ match สี overlap กับ ref_cam (front)
#         """
#         if self.latest_images[side_cam] is None:
#             return

#         base_img = cv2.resize(self.latest_images[side_cam], self.target_size)

#         best_scale = 1.0
#         best_score = 999999999.0
#         scale_candidates = np.linspace(0.9, 1.1, 21)  # สเกลตั้งแต่ 0.9 ถึง 1.1

#         for s in scale_candidates:
#             zoomed = self.zoom_image(base_img, s)
#             score = self.compare_overlap(zoomed, self.latest_images[ref_cam], side_cam, ref_cam)
#             if score < best_score:
#                 best_score = score
#                 best_scale = s

#         final_img = self.zoom_image(base_img, best_scale, remove_black=True)
#         self.latest_images[side_cam] = final_img
#         print(f"{side_cam} best_scale={best_scale:.2f}, score={best_score:.2f}")

#     def split_and_adjust_back(self):
#         """
#         แบ่งภาพ back เป็น 2 ซีก: back_left, back_right
#         แล้วปรับ scale โดยอิง left, right
#         """
#         if self.latest_images['back'] is None:
#             return

#         back_img = cv2.resize(self.latest_images['back'], self.target_size)
#         w = back_img.shape[1]
#         half_w = w // 2

#         back_left_img = back_img[:, :half_w]
#         back_right_img = back_img[:, half_w:]

#         best_scale_l = 1.0
#         best_score_l = 999999999.0
#         scale_candidates = np.linspace(0.9, 1.1, 21)

#         # back_left match กับ left
#         for s in scale_candidates:
#             zoomed = self.zoom_image(back_left_img, s)
#             score = self.compare_overlap(zoomed, self.latest_images['left'], 'back_left', 'left')
#             if score < best_score_l:
#                 best_score_l = score
#                 best_scale_l = s

#         final_bl = self.zoom_image(back_left_img, best_scale_l, remove_black=True)
#         print(f"back_left best_scale={best_scale_l:.2f}, score={best_score_l:.2f}")

#         # back_right match กับ right
#         best_scale_r = 1.0
#         best_score_r = 999999999.0
#         for s in scale_candidates:
#             zoomed = self.zoom_image(back_right_img, s)
#             score = self.compare_overlap(zoomed, self.latest_images['right'], 'back_right', 'right')
#             if score < best_score_r:
#                 best_score_r = score
#                 best_scale_r = s

#         final_br = self.zoom_image(back_right_img, best_scale_r, remove_black=True)
#         print(f"back_right best_scale={best_scale_r:.2f}, score={best_score_r:.2f}")

#         # รวม back_left + back_right
#         self.latest_images['back'] = np.hstack((final_bl, final_br))

#     def zoom_image(self, img, scale, remove_black=False):
#         """
#         zoom in/out ด้วย factor = scale
#         remove_black=True จะครอปขอบดำออกก่อน resize กลับเป็น (640,480)
#         """
#         h, w = img.shape[:2]
#         new_w = int(w * scale)
#         new_h = int(h * scale)
#         zoomed = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_CUBIC)

#         if remove_black:
#             # ตัดขอบดำ (pixel=0) ออก
#             gray = cv2.cvtColor(zoomed, cv2.COLOR_BGR2GRAY)
#             _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
#             contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#             if contours:
#                 x, y, cw, ch = cv2.boundingRect(contours[0])
#                 zoomed = zoomed[y:y+ch, x:x+cw]

#         # resize สุดท้ายกลับเป็น 640×480
#         zoomed = cv2.resize(zoomed, (640, 480), interpolation=cv2.INTER_CUBIC)
#         return zoomed

#     def compare_overlap(self, imgA, imgB, camA, camB):
#         """คำนวณคะแนนความต่างสีในบริเวณ overlap"""
#         overlap_size = 50

#         if (camA == 'left' and camB == 'front'):
#             partA = imgA[:, -overlap_size:]
#             partB = imgB[:, :overlap_size]
#         elif (camA == 'right' and camB == 'front'):
#             partA = imgA[:, :overlap_size]
#             partB = imgB[:, -overlap_size:]
#         elif (camA == 'back_left' and camB == 'left'):
#             partA = imgA[:, -overlap_size:]
#             partB = imgB[:, :overlap_size]
#         elif (camA == 'back_right' and camB == 'right'):
#             partA = imgA[:, :overlap_size]
#             partB = imgB[:, -overlap_size:]
#         else:
#             partA = imgA[:, :overlap_size]
#             partB = imgB[:, :overlap_size]

#         # ตัดขอบโค้งออกเล็กน้อย เพื่อความ smooth (ตัวอย่าง 10 px)
#         side_crop = 10
#         partA = partA[side_crop:-side_crop, :]
#         partB = partB[side_crop:-side_crop, :]

#         # resize ส่วน overlap ให้เท่ากัน
#         partA = cv2.resize(partA, (overlap_size, 480 - side_crop*2))
#         partB = cv2.resize(partB, (overlap_size, 480 - side_crop*2))

#         diff = cv2.absdiff(partA, partB)
#         return float(np.mean(diff))

#     def create_panorama(self):
#         """ตัด overlap + ตัดส่วนโค้งด้านข้าง + stitch ภาพ"""
#         timestamp = int(time.time())

#         left = self.latest_images['left']
#         front = self.latest_images['front']
#         right = self.latest_images['right']
#         back = self.latest_images['back']

#         if any(x is None for x in [left, front, right, back]):
#             print("Some images are missing after scale adjustment.")
#             return

#         overlap = 50
#         w_b = back.shape[1]
#         half_w = w_b // 2
#         back_left = back[:, :half_w]
#         back_right = back[:, half_w:]

#         # ตัดส่วนโค้งอีกหน่อย (เช่น 20px)
#         def remove_side_curvature(img, crop=20):
#             # ตัดซ้ายขวา
#             return img[:, crop:-crop]

#         # ตัด Overlap
#         back_left = back_left[:, :-overlap]
#         left = left[:, :-overlap]
#         front = front[:, overlap:-overlap]
#         right = right[:, overlap:]
#         back_right = back_right[:, overlap:]

#         # remove curvature
#         back_left = remove_side_curvature(back_left)
#         left = remove_side_curvature(left)
#         front = remove_side_curvature(front)
#         right = remove_side_curvature(right)
#         back_right = remove_side_curvature(back_right)

#         # ทำให้ความสูงเท่ากันอีกครั้ง เผื่อโดนตัดขนาดไม่เท่ากัน
#         min_h = min(img.shape[0] for img in [back_left, left, front, right, back_right])
#         def fit_height(img):
#             return cv2.resize(img, (int(img.shape[1] * min_h / img.shape[0]), min_h))

#         back_left = fit_height(back_left)
#         left      = fit_height(left)
#         front     = fit_height(front)
#         right     = fit_height(right)
#         back_right= fit_height(back_right)

#         # ต่อภาพ
#         panorama = np.hstack((back_left, left, front, right, back_right))

#         pano_name = f"panorama_{timestamp}.png"
#         pano_path = os.path.join(self.output_folder, pano_name)
#         cv2.imwrite(pano_path, panorama)
#         print(f"Saved final panorama => {pano_path}")

# if __name__ == "__main__":
#     ZoomPanoramaGenerator()
#!/usr/bin/env python3
import cv2
import numpy as np
import os
import time
import glob

class CleanedRevertedPanoramaGenerator:
    def __init__(self):
        self.image_paths = {
            'front': 'reverted_images/front_reverted.png',
            'back':  'reverted_images/back_reverted.png',
            'left':  'reverted_images/left_reverted.png',
            'right': 'reverted_images/right_reverted.png'
        }
        self.output_folder = "cleaned_reverted_panorama"
        os.makedirs(self.output_folder, exist_ok=True)

        # Load images
        self.latest_images = self.load_latest_images()

        # Check if all images are available
        if any(img is None for img in self.latest_images.values()):
            print("Error: Some images could not be loaded.")
            return

        # Remove black edges before further processing
        for cam in self.latest_images:
            self.latest_images[cam] = self.remove_black_borders(self.latest_images[cam])

        # Resize front image as reference
        self.target_size = (640, 480)
        self.latest_images['front'] = cv2.resize(self.latest_images['front'], self.target_size)

        # Adjust scale for side images
        self.adjust_scale_side('left', 'front')
        self.adjust_scale_side('right', 'front')

        # Split back image without flipping
        self.split_and_adjust_back()

        # Create final panorama
        self.create_panorama()

    def load_latest_images(self):
        """Load latest reverted images from folder"""
        latest_images = {}
        for cam, pattern in self.image_paths.items():
            files = sorted(glob.glob(pattern), key=os.path.getctime, reverse=True)
            if files:
                latest_images[cam] = cv2.imread(files[0])
                print(f"Loaded {files[0]} for {cam}.")
            else:
                print(f"Warning: No file found for {cam}.")
                latest_images[cam] = None
        return latest_images

    def remove_black_borders(self, img):
        """Remove black edges from the image"""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            x, y, w, h = cv2.boundingRect(contours[0])
            img = img[y:y+h, x:x+w]
        
        return cv2.resize(img, (640, 480), interpolation=cv2.INTER_CUBIC)

    def adjust_scale_side(self, side_cam, ref_cam):
        """Adjust scale for side cameras to match front"""
        if self.latest_images[side_cam] is None:
            return

        base_img = cv2.resize(self.latest_images[side_cam], self.target_size)
        self.latest_images[side_cam] = base_img

    def split_and_adjust_back(self):
        """Split back image into left and right sections without flipping"""
        if self.latest_images['back'] is None:
            return

        back_img = cv2.resize(self.latest_images['back'], self.target_size)
        w = back_img.shape[1] // 2

        # Split into left and right without flipping
        self.latest_images['back_left'] = back_img[:, :w]
        self.latest_images['back_right'] = back_img[:, w:]

    def create_panorama(self):
        """Stitch images into a panorama"""
        timestamp = int(time.time())

        left = self.latest_images['left']
        front = self.latest_images['front']
        right = self.latest_images['right']
        back_left = self.latest_images.get('back_left', None)
        back_right = self.latest_images.get('back_right', None)

        if any(x is None for x in [left, front, right, back_left, back_right]):
            print("Some images are missing after processing.")
            return

        # Swap order of back_left and back_right to correct the alignment
        panorama = np.hstack((back_right, left, front, right, back_left))

        pano_name = f"panorama.png"
        pano_path = os.path.join(self.output_folder, pano_name)
        cv2.imwrite(pano_path, panorama)
        print(f"Saved final cleaned panorama => {pano_path}")

if __name__ == "__main__":
    CleanedRevertedPanoramaGenerator()
