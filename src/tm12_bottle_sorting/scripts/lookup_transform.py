#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def transform_point(listener, point, from_frame, to_frame):
    # Set the frame_id of the point to the original (camera) frame
    point.header.frame_id = from_frame
    try:
        # Wait for the transform to be available
        listener.waitForTransform(to_frame, from_frame, rospy.Time(), rospy.Duration(4.0))
        # Transform the point from the camera frame to the base frame
        transformed_point = listener.transformPose(to_frame, point)
        return transformed_point
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Transformation error: {}".format(e))
        return None

def main():
    # Initialize the ROS node
    rospy.init_node('coordinate_transform', anonymous=True)

    # Initialize the TF listener
    listener = tf.TransformListener()

    # Define the point in the camera frame (tool0)
    camera_point = PoseStamped()
    camera_point.pose.position.x = 0.07977208321586733
    camera_point.pose.position.y = 0.05949095348616305
    camera_point.pose.position.z = 0.3920000186190009

    camera_point.pose.orientation.w = 1.0

    # Transform the point from the camera frame (tool0) to the base frame
    base_point = transform_point(listener, camera_point, 'tool0', 'base')
    if base_point:
        rospy.loginfo("Point in base frame: x={}, y={}, z={}".format(
            base_point.pose.position.x, base_point.pose.position.y, base_point.pose.position.z))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
