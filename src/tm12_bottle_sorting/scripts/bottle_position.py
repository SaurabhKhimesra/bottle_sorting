#!/usr/bin/env python3

import os

import rospy
import rospkg
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


def runtime_dir():
    """Directory the captured frames and the bottle slot are written to.

    Defaults to runtime/ inside the package so a fresh clone works without
    configuration; override with the runtime_dir parameter to point it at a
    tmpfs or a shared folder.
    """
    default = os.path.join(rospkg.RosPack().get_path('tm12_bottle_sorting'), 'runtime')
    path = rospy.get_param('~runtime_dir', default)
    os.makedirs(path, exist_ok=True)
    return path

class RGBDImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.runtime_dir = runtime_dir()
        camera = rospy.get_param('~camera', '/cam_1')
        rospy.Subscriber(camera + '/color/image_raw', Image, self.rgb_callback)
        rospy.Subscriber(camera + '/depth/image_rect_raw', Image, self.depth_callback)
        rospy.Subscriber('robot_feedback', String, self.feedback_callback)
        self.feedback_pub = rospy.Publisher('gripper_feedback', String, queue_size=10)
        self.position_reached = False

    def rgb_callback(self, data):
        self.rgb_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, '16UC1')

    def feedback_callback(self, msg):
        if msg.data == "Robot has reached the position to detect the bottle":
            self.position_reached = True
            self.save_images()

    def save_images(self):
        if self.rgb_image is not None and self.depth_image is not None:
            cv2.imwrite(os.path.join(self.runtime_dir, 'rgb_image.jpg'), self.rgb_image)
            cv2.imwrite(os.path.join(self.runtime_dir, 'depth_image.png'), self.depth_image)
            rospy.loginfo("Saved RGB and depth frames to %s", self.runtime_dir)
            self.process_images()

    def process_images(self):
        rgb_image_path = os.path.join(self.runtime_dir, 'rgb_image.jpg')
        detected_bottle_coords, processed_image = detect_hough_circles(rgb_image_path)
        bottle_positions = get_bottle_position(detected_bottle_coords)
        
        if bottle_positions:
            min_position = min(bottle_positions)
            rospy.loginfo("Minimum bottle position: %s", min_position)
            with open(os.path.join(self.runtime_dir, 'min_bottle_position.txt'), 'w') as file:
                file.write(str(min_position))
            
            # Publish message to indicate coordinates are being sent to the gripper
            msg = String()
            msg.data = "sending co-ordinates of the detected bottle to the gripper"
            self.feedback_pub.publish(msg)
        
        rospy.loginfo("Detected bottle coordinates: %s", detected_bottle_coords)
        rospy.loginfo("Crate slots occupied: %s", bottle_positions)

        # Always write the annotated frame; only pop a window up when asked,
        # since waitKey blocks the callback and there is no display under
        # roslaunch.
        cv2.imwrite(os.path.join(self.runtime_dir, 'detected_bottle.png'), processed_image)
        if rospy.get_param('~show_debug_window', False):
            cv2.imshow("Hough Circles", processed_image)
            cv2.waitKey(1)

    def run(self):
        rospy.init_node('rgbd_image_saver', anonymous=True)
        rospy.loginfo("RGB-D Image capture node initialized and waiting for feedback message.")
        rospy.spin()

def detect_hough_circles(rgb_image_path):
    rgb_image = cv2.imread(rgb_image_path)
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.medianBlur(gray, 5)
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=40, param1=80, param2=20, minRadius=16, maxRadius=22)

    detected_bottle_coords = []
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(rgb_image, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(rgb_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            detected_bottle_coords.append((x, y))
    
    return detected_bottle_coords, rgb_image

def get_bottle_position(detected_bottle_coords):
    y_ranges = [(0, 155), (156, 248), (249, 345), (346, 480)]
    x_ranges = [(117, 240), (241, 335), (336, 425), (426, 565)]
    positions = []
    for (x, y) in detected_bottle_coords:
        for y_idx, y_range in enumerate(y_ranges):
            if y_range[0] <= y <= y_range[1]:
                for x_idx, x_range in enumerate(x_ranges):
                    if x_range[0] <= x <= x_range[1]:
                        position_number = 1 + x_idx + (y_idx * 4)
                        positions.append(position_number)
                        break
                break
    return positions

if __name__ == '__main__':
    saver = RGBDImageSaver()
    saver.run()
