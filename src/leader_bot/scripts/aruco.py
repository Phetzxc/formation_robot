# import cv2
# import numpy as np
# import os

# # Define the ArUco marker size in meters (e.g., 0.05 for a 5 cm marker)
# marker_length = 0.12  # Adjust this based on the real marker size

# # Define the camera intrinsic matrix (example values, replace with actual calibration)
# camera_matrix = np.array([[800, 0, 320],
#                           [0, 800, 240],
#                           [0, 0, 1]], dtype=np.float32)

# # Define distortion coefficients (example values, replace with actual calibration)
# dist_coeffs = np.zeros((5, 1), dtype=np.float32)

# def process_image(image_source):
#     image_name = os.path.splitext(os.path.basename(image_source))[0]
#     os.makedirs(image_name, exist_ok=True)

#     aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

#     parameters = cv2.aruco.DetectorParameters()
#     detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

#     image = cv2.imread(image_source)
#     if image is None:
#         print(f"Error: Cannot open image {image_source}")
#         return

#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     corners, ids, _ = detector.detectMarkers(gray)

#     detected_ids = []  # Store detected marker IDs

#     if ids is not None:
#         cv2.aruco.drawDetectedMarkers(image, corners, ids)

#         # Estimate pose of each detected marker
#         rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

#         for i, (marker_id_arr, corner) in enumerate(zip(ids, corners)):
#             marker_id = marker_id_arr[0]
#             detected_ids.append(marker_id)  # Store detected marker ID
            
#             cx, cy = np.mean(corner[0][:, 0]), np.mean(corner[0][:, 1])

#             # Draw text with larger font size
#             font = cv2.FONT_HERSHEY_SIMPLEX
#             cv2.putText(image, f"ID: {marker_id}", (int(cx), int(cy) - 10), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

#             # Draw coordinate axes
#             axis_length = 50
#             cv2.arrowedLine(image, (int(cx), int(cy)), (int(cx) + axis_length, int(cy)), (255, 0, 0), 2)  # X-axis (Blue)
#             cv2.arrowedLine(image, (int(cx), int(cy)), (int(cx), int(cy) + axis_length), (0, 255, 0), 2)  # Y-axis (Green)
#             cv2.arrowedLine(image, (int(cx), int(cy)), (int(cx) - axis_length, int(cy) - axis_length), (0, 0, 255), 2)  # Z-axis (Red)

#             # Draw corner points of the ArUco marker
#             for j, (x, y) in enumerate(corner[0]):
#                 cv2.circle(image, (int(x), int(y)), 5, (255, 255, 0), -1)
#                 cv2.putText(image, f"{j}", (int(x) + 5, int(y) - 5), font, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

#             # Pose estimation
#             rvec, tvec = rvecs[i], tvecs[i]
#             x, y, z = tvec[0]

#             # Convert rotation vector to a rotation matrix
#             R, _ = cv2.Rodrigues(rvec)
#             yaw = np.arctan2(R[1, 0], R[0, 0])  # Extract yaw angle in radians
#             pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))  # Extract pitch angle in radians
#             roll = np.arctan2(R[2, 1], R[2, 2])  # Extract roll angle in radians

#             # Display the position and rotation angles
#             pose_text = f"X: {x:.2f}m Y: {y:.2f}m Z: {z:.2f}m\nYaw: {yaw:.3f} rad Pitch: {pitch:.3f} rad Roll: {roll:.3f} rad"
#             for j, line in enumerate(pose_text.split('\n')):
#                 cv2.putText(image, line, (int(cx), int(cy) + 30 + j*30), font, 0.7, (0, 165, 255), 2, cv2.LINE_AA)

#             file_name = f"Marker_{marker_id}.png"
#             save_path = os.path.join(image_name, file_name)
#             cv2.imwrite(save_path, image)
#             print(f"Saved: {save_path} with Pose: {pose_text}")

#     print(f"Processing finished for {image_source}")
#     if detected_ids:
#         print(f"Detected Marker IDs: {detected_ids}")
#     else:
#         print("No markers detected.")

# def main(image_sources):
#     for img in image_sources:
#         process_image(img)

# if __name__ == "__main__":
#     images = [
#         r"/home/phet/Downloads/aruco-marker-ID1.webp"   
#     ]
    
#     main(images)


import cv2
import numpy as np
import os

# Define the ArUco marker size in meters (e.g., 0.12 for a 12 cm marker)
marker_length = 0.12  

# Camera intrinsic matrix (replace with actual calibration)
camera_matrix = np.array([[800, 0, 320],
                          [0, 800, 240],
                          [0, 0, 1]], dtype=np.float32)

# Distortion coefficients (replace with actual calibration)
dist_coeffs = np.zeros((5, 1), dtype=np.float32)

def process_image(image_source):
    image_name = os.path.splitext(os.path.basename(image_source))[0]
    os.makedirs(image_name, exist_ok=True)

    # Use the correct ArUco dictionary (Original ArUco)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    image = cv2.imread(image_source)
    if image is None:
        print(f"Error: Cannot open image {image_source}")
        return

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        # Estimate pose of each detected marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        for i, (marker_id_arr, corner) in enumerate(zip(ids, corners)):
            marker_id = marker_id_arr[0]
            
            # Compute center of the marker
            cx, cy = np.mean(corner[0][:, 0]), np.mean(corner[0][:, 1])

            # Draw text with larger font size
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image, f"ID: {marker_id}", (int(cx), int(cy) - 10), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Pose estimation
            rvec, tvec = rvecs[i], tvecs[i]
            x, y, z = tvec[0]

            # Convert rotation vector to a rotation matrix
            R, _ = cv2.Rodrigues(rvec)

            # Compute proper Euler angles
            yaw = np.arctan2(R[1, 0], R[0, 0])  # Yaw (rotation around Z)
            pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))  # Pitch (rotation around Y)
            roll = np.arctan2(R[2, 1], R[2, 2])  # Roll (rotation around X)

            # Normalize angles close to -π or π
            if np.isclose(roll, -np.pi, atol=0.05):
                roll = 0.0

            # Display the position and rotation angles
            pose_text = f"X: {x:.2f}m Y: {y:.2f}m Z: {z:.2f}m\nYaw: {yaw:.3f} rad Pitch: {pitch:.3f} rad Roll: {roll:.3f} rad"
            for j, line in enumerate(pose_text.split('\n')):
                cv2.putText(image, line, (int(cx), int(cy) + 30 + j*30), font, 0.7, (0, 165, 255), 2, cv2.LINE_AA)

            # Draw the correct coordinate axes
            cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, marker_length * 0.5)

            # Save the result
            file_name = f"Marker_{marker_id}.png"
            save_path = os.path.join(image_name, file_name)
            cv2.imwrite(save_path, image)
            print(f"Saved: {save_path} with Pose: {pose_text}")

    print(f"Processing finished for {image_source}")

def main(image_sources):
    for img in image_sources:
        process_image(img)

if __name__ == "__main__":
    images = [r"/home/phet/Downloads/aruco-marker-ID1.webp" ]  # Path to your uploaded image
    main(images)
