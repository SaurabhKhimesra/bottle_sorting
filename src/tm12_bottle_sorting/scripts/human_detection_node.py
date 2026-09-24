#!/usr/bin/env python3

import sys

import rospy
import cv2
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs
import moveit_commander
import moveit_msgs.msg
import shape_msgs.msg
import geometry_msgs.msg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Initialize RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

# Camera intrinsic parameters (hardcoded for now)
fx = 606.5270385742188
fy = 605.458984375
cx = 326.5716247558594
cy = 244.11395263671875

# MediaPipe pose/landmark detection
holistic = mp.solutions.holistic.Holistic(min_detection_confidence=0.5,
                                          min_tracking_confidence=0.5)

# Robot control parameters
bridge = CvBridge()
current_joint_positions = [0.0] * 6
tolerance = 0.05  # 5% of the target joint angle

# MoveIt! Initialization
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"  # Replace with your group name
move_group = moveit_commander.MoveGroupCommander(group_name)
planning_frame = move_group.get_planning_frame()
eef_link = move_group.get_end_effector_link()
group_names = robot.get_group_names()

def image_callback(msg):
    global move_group, scene

    # Convert ROS Image message to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Perform human detection using MediaPipe
    results = holistic.process(cv_image)
    human_detected = results.pose_landmarks is not None

    # Get depth frame
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    depth_image = np.asanyarray(depth_frame.get_data())

    if human_detected:
        # Slow down the robot or stop it
        rospy.loginfo("Human detected. Slowing down or stopping the robot.")
        move_group.stop()

    else:
        # Detect obstacles using depth data
        depth_threshold = 1.0  # Threshold distance in meters
        obstacle_detected = np.any(depth_image < (depth_threshold / depth_scale))

        if obstacle_detected:
            # Replan path
            rospy.loginfo("Obstacle detected. Replanning path...")

            # Update planning scene with the obstacle
            collision_object = moveit_msgs.msg.CollisionObject()
            collision_object.header.frame_id = robot.get_planning_frame()
            collision_object.id = "obstacle"

            primitive = shape_msgs.msg.SolidPrimitive()
            primitive.type = primitive.BOX
            primitive.dimensions = [0.1, 0.1, 0.1]  # Adjust dimensions as necessary

            obstacle_pose = geometry_msgs.msg.Pose()
            obstacle_pose.orientation.w = 1.0
            obstacle_pose.position.x = 0.5  # Example values
            obstacle_pose.position.y = 0.5
            obstacle_pose.position.z = 0.5

            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(obstacle_pose)
            collision_object.operation = collision_object.ADD

            scene.add_object(collision_object)

            # Plan a new path
            move_group.set_start_state_to_current_state()
            new_plan = move_group.plan()

            # Execute the new plan
            if new_plan:
                rospy.loginfo("Executing the new plan.")
                move_group.execute(new_plan, wait=True)
            else:
                rospy.logwarn("No valid plan found.")

def main():
    rospy.init_node('human_detection_node', anonymous=True)
    
    # Subscribe to the camera feed
    image_topic = rospy.get_param('~image_topic', '/cam_1/color/image_raw')
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.loginfo("Human detection watching %s", image_topic)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
