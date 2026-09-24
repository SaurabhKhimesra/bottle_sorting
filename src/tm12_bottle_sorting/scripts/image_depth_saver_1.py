#!/usr/bin/env python3
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from sensor_msgs.msg import Image
import message_filters


def runtime_dir():
    """Where captured frames are written. Override with the runtime_dir param."""
    default = os.path.join(rospkg.RosPack().get_path('tm12_bottle_sorting'), 'runtime')
    path = rospy.get_param('~runtime_dir', default)
    os.makedirs(path, exist_ok=True)
    return path

# Initialize a flag to check if the image has been saved
image_saved = False
saved_cv_image = None
saved_depth_image = None

def callback(color_msg, depth_msg):
    global image_saved, saved_cv_image, saved_depth_image
    bridge = CvBridge()
    
    if not image_saved:
        # Convert your ROS Image message to OpenCV2 format only once
        try:
            saved_cv_image = bridge.imgmsg_to_cv2(color_msg, "bgr8")
            saved_depth_image = bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except CvBridgeError as e:
            print(e)
            return

        save_path = runtime_dir()

        # Fixed filenames for the images
        image_name = 'rgb1.jpeg'
        depth_name = 'depth1.png'

        # Save RGB and depth images
        cv2.imwrite(os.path.join(save_path, image_name), saved_cv_image)
        cv2.imwrite(os.path.join(save_path, depth_name), saved_depth_image)

        # Set the flag to True indicating the images have been saved
        image_saved = True

    # Always publish the saved images
    if saved_cv_image is not None and saved_depth_image is not None:
        pub_depth.publish(bridge.cv2_to_imgmsg(saved_depth_image, encoding="passthrough"))
        pub_color.publish(bridge.cv2_to_imgmsg(saved_cv_image, encoding="bgr8"))

def main():
    rospy.init_node('image_depth_saver', anonymous=True)

    # Create subscribers for both color and depth images
    global image_sub, depth_sub, pub_color, pub_depth
    image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)

    # Create publishers for both color and depth images
    pub_color = rospy.Publisher("1RGB_photo", Image, queue_size=10)
    pub_depth = rospy.Publisher("1D_depth_photo", Image, queue_size=10)
    
    # Synchronize the subscribers by time
    ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
    ts.registerCallback(callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()
