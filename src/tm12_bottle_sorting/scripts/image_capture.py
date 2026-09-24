#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os

class ImageCapture:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.count = 0
        os.makedirs('calibration_images', exist_ok=True)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
        key = cv2.waitKey(3)

        if key == ord('c'):
            img_name = f"calibration_images/image_{self.count}.jpg"
            cv2.imwrite(img_name, cv_image)
            rospy.loginfo(f"Saved {img_name}")
            self.count += 1
        elif key == ord('q'):
            cv2.destroyAllWindows()
            rospy.signal_shutdown('Quit')

def main():
    ic = ImageCapture()
    rospy.init_node('image_capture', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
