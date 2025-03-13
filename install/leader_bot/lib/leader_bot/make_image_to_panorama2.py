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

        # Split back image and adjust scales
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
        """Split back image into left and right sections and adjust"""
        if self.latest_images['back'] is None:
            return

        back_img = cv2.resize(self.latest_images['back'], self.target_size)
        w = back_img.shape[1] // 2

        # สลับตำแหน่ง back_left และ back_right ให้ถูกต้อง
        self.latest_images['back_right'] = back_img[:, :w]
        self.latest_images['back_left'] = back_img[:, w:]

    def remove_curved_edges(self, img):
        """Crop curved edges to improve stitching"""
        h, w = img.shape[:2]
        crop_x = int(w * 0.02)  # Crop 2% from sides
        crop_y = int(h * 0.05)  # Crop 5% from top and bottom
        return img[crop_y:-crop_y, crop_x:-crop_x]

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

        # สลับตำแหน่ง back_left และ back_right เพื่อให้เชื่อมต่อถูกต้อง
        panorama = np.hstack((back_left, left, front, right, back_right))

        # ตัดขอบโค้งออกให้ดูเนียนขึ้น
        panorama = self.remove_curved_edges(panorama)

        pano_name = f"panorama2.png"
        pano_path = os.path.join(self.output_folder, pano_name)
        cv2.imwrite(pano_path, panorama)
        print(f"Saved final cleaned panorama => {pano_path}")

if __name__ == "__main__":
    CleanedRevertedPanoramaGenerator()
