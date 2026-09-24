#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image

def callback(rgb_msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    img, bottlesAmount = detectBottles(cv_image)
    window_title = "Detected Bottles: " + str(bottlesAmount)
    cv2.imshow(window_title, img)
    cv2.waitKey(1)  # Use waitKey(1) to allow GUI events to process

def detectBottles(cv_image):
    imgGr = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    imgGr = clahe.apply(imgGr)
    imgGrBlur = cv2.GaussianBlur(imgGr, (9, 9), 0)

    bottles = cv2.HoughCircles(imgGrBlur, cv2.HOUGH_GRADIENT, 1, 20,
                               param1=50, param2=30, minRadius=10, maxRadius=22)
    if bottles is not None:
        bottles = np.uint16(np.around(bottles))
        for x, y, r in bottles[0, :]:
            cv2.circle(cv_image, (x, y), r, (0, 255, 0), 2)  # Draw the circle around the bottle
            cv2.circle(cv_image, (x, y), 2, (0, 0, 255), 3)  # Draw the center of the circle
        bottlesAmount = len(bottles[0])
    else:
        bottlesAmount = 0
    return cv_image, bottlesAmount

def main():
    rospy.init_node('bottle_detector', anonymous=True)
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, callback)
    rospy.spin()  # Keep the program alive

if __name__ == '__main__':
    main()
