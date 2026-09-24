#!/usr/bin/env python3
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import message_filters
from sensor_msgs.msg import Image


def runtime_dir():
    """Where captured frames are written. Override with the runtime_dir param."""
    default = os.path.join(rospkg.RosPack().get_path('tm12_bottle_sorting'), 'runtime')
    path = rospy.get_param('~runtime_dir', default)
    os.makedirs(path, exist_ok=True)
    return path


def callback(color_msg, depth_msg):
    bridge = CvBridge()
    try:
        # Convert your ROS Image message to OpenCV2 format
        cv_image = bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth_image = bridge.imgmsg_to_cv2(depth_msg, "passthrough")  # 'passthrough' preserves the depth data
    except CvBridgeError as e:
        print(e)
    else:
        save_path = runtime_dir()

        # Create filenames with timestamp to avoid overwriting
        timestamp = rospy.Time.now()
        image_name = 'bottle_rgb_{}.jpeg'.format(timestamp)
        depth_name = 'bottle_depth_{}.png'.format(timestamp)

        # Save RGB and depth images
        cv2.imwrite(os.path.join(save_path, image_name), cv_image)
        cv2.imwrite(os.path.join(save_path, depth_name), depth_image)  # Save depth image as PNG

def main():
    rospy.init_node('image_depth_saver', anonymous=True)
    # Create subscribers for both color and depth images
    image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
    
    # Synchronize the subscribers by time
    ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
    ts.registerCallback(callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()
