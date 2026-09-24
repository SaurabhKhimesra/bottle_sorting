#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2
import os

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Define alignment process to align depth frames to color frames
align = rs.align(rs.stream.color)

# Hardcoded intrinsic parameters from the camera_info topic
fx = 606.5270385742188
fy = 605.458984375
cx = 326.5716247558594
cy = 244.11395263671875

# Hardcoded extrinsic parameters from the depth_to_color topic
rotation_matrix = np.array([
    [0.9998409748077393, -0.0165542159229517, 0.006636003032326698],
    [0.016562694683670998, 0.9998620748519897, -0.001224879059009254],
    [-0.006614810787141323, 0.0013345943298190832, 0.9999772310256958]
])
translation_vector = np.array([0.015260392799973488, 1.0333438694942743e-05, 0.0001475200115237385])

depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

# Path to save images. This script drives the camera directly rather than
# running as a ROS node, so it resolves the directory relative to itself.
save_path = os.environ.get(
    'TM12_RUNTIME_DIR',
    os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, 'runtime'))
os.makedirs(save_path, exist_ok=True)

# Gripper transformation parameters (provided)
gripper_translation = np.array([717.360, 38.198, 389.279])
gripper_rotation_degrees = np.array([-178.869, 1.051, 80.986])

# Convert rotation angles from degrees to radians
gripper_rotation_radians = np.deg2rad(gripper_rotation_degrees)

# Create rotation matrices
Rx = np.array([
    [1, 0, 0],
    [0, np.cos(gripper_rotation_radians[0]), -np.sin(gripper_rotation_radians[0])],
    [0, np.sin(gripper_rotation_radians[0]), np.cos(gripper_rotation_radians[0])]
])

Ry = np.array([
    [np.cos(gripper_rotation_radians[1]), 0, np.sin(gripper_rotation_radians[1])],
    [0, 1, 0],
    [-np.sin(gripper_rotation_radians[1]), 0, np.cos(gripper_rotation_radians[1])]
])

Rz = np.array([
    [np.cos(gripper_rotation_radians[2]), -np.sin(gripper_rotation_radians[2]), 0],
    [np.sin(gripper_rotation_radians[2]), np.cos(gripper_rotation_radians[2]), 0],
    [0, 0, 1]
])

# Combined rotation matrix for the gripper
gripper_rotation_matrix = np.dot(np.dot(Rz, Ry), Rx)

try:
    image_saved = False  # Flag to check if image has been saved
    while not image_saved:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Image processing for bottle detection
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray_image, (9, 9), 0)
        circles = cv2.HoughCircles(gray_blur, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=22)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center_x, center_y = i[0], i[1]
                # Draw the circle in the output image
                cv2.circle(color_image, (center_x, center_y), i[2], (0, 255, 0), 2)
                cv2.circle(color_image, (center_x, center_y), 2, (0, 0, 255), 3)  # center of the circle

                # Get depth value from depth image at the center of the bottle
                depth_value = depth_image[center_y, center_x] * depth_scale  # Convert to meters if necessary

                if depth_value == 0:
                    print(f"Depth value at ({center_x}, {center_y}) is zero, skipping this point.")
                    continue

                # Convert to 3D world coordinates using intrinsic parameters
                X = (center_x - cx) * depth_value / fx
                Y = (center_y - cy) * depth_value / fy
                Z = depth_value

                # Print camera coordinates
                print(f"Camera coordinates: X={X}, Y={Y}, Z={Z}")

                # Apply extrinsic transformation from depth to color camera
                point_3D_camera = np.array([X, Y, Z])
                point_3D_transformed_camera = np.dot(rotation_matrix, point_3D_camera) + translation_vector

                # Apply transformation to gripper coordinates
                point_3D_gripper = np.dot(gripper_rotation_matrix, point_3D_transformed_camera) + gripper_translation

                # Print gripper coordinates
                print(f"Gripper coordinates: X={point_3D_gripper[0]}, Y={point_3D_gripper[1]}, Z={point_3D_gripper[2]}")

                # Save the image with detected bottle
                image_name = "detected_bottle.png"
                cv2.imwrite(os.path.join(save_path, image_name), color_image)
                print(f"Image saved as {image_name} at {save_path}")
                image_saved = True  # Update flag to true as image is saved
                break  # Exit the loop after saving the image

        if image_saved:
            cv2.imshow('Detected Bottles', color_image)
            cv2.waitKey(0)  # Wait for a key press to close the window
            cv2.destroyAllWindows()
finally:
    # Stop streaming
    pipeline.stop()
