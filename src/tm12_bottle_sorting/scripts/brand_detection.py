#!/usr/bin/env python3

import cv2
import os
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

# Initialize ROS node
rospy.init_node('rgbd_camera_yolo', anonymous=True)

# Create a CV bridge to convert ROS images to OpenCV format
bridge = CvBridge()

# Paths. yolov5 is a submodule of this workspace, so it sits alongside the
# package rather than inside it; both are overridable from the launch file.
_pkg_dir = rospkg.RosPack().get_path('tm12_bottle_sorting')
image_save_path = rospy.get_param('~runtime_dir', os.path.join(_pkg_dir, 'runtime'))
yolo_script_path = rospy.get_param('~yolov5_dir', os.path.join(_pkg_dir, os.pardir, 'yolov5'))
weights_path = rospy.get_param(
    '~weights', os.path.join(yolo_script_path, 'runs', 'train', 'exp', 'weights', 'best.pt'))
output_directory = 'static_output'  # Consistent output directory for YOLOv5

os.makedirs(image_save_path, exist_ok=True)

# Publisher for the class ID and detection message
result_publisher = rospy.Publisher('bottle_sorting_topic', String, queue_size=10)
detection_message_publisher = rospy.Publisher('detection_message_topic', String, queue_size=10)

def handle_single_image(msg):
    try:
        # Convert ROS image to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image_256 = cv2.resize(cv_image, (256, 256), interpolation=cv2.INTER_LINEAR)

        # Save the image
        img_filename = "camera_image_256.jpg"
        img_filepath = os.path.join(image_save_path, img_filename)
        cv2.imwrite(img_filepath, cv_image_256)
        rospy.loginfo(f"Image saved at {img_filepath}")

        # YOLOv5 detection
        detection_command = (
            f"python3 {yolo_script_path}/detect.py --weights {weights_path} "
            f"--img 256 --conf 0.25 --source {img_filepath} --save-txt --name {output_directory}")
        rospy.loginfo(f"Running detection command: {detection_command}")
        os.system(detection_command)

        # Read the YOLOv5 output and process it
        result_file = os.path.join(yolo_script_path, f'runs/detect/{output_directory}/labels', img_filename.replace('.jpg', '.txt'))
        rospy.loginfo(f"Result file path: {result_file}")

        if os.path.exists(result_file):
            rospy.loginfo("Result file found.")
            with open(result_file, 'r') as file:
                class_ids = [line.split()[0] for line in file]
                if class_ids:
                    rospy.loginfo(f"Class IDs detected: {class_ids}")
                    # Write only the class ID to a new simplified result file
                    simplified_result_file = os.path.join(image_save_path, 'simplified_result.txt')
                    with open(simplified_result_file, 'w') as sfile:
                        sfile.write('\n'.join(class_ids))

                    # Publish the first class ID
                    result_publisher.publish(class_ids[0])
                    rospy.loginfo(f"Published class ID: {class_ids[0]}")

                    # Publish the detection message
                    detection_message_publisher.publish("Brand of the bottle has been detected")
                    rospy.loginfo("Published detection message.")
        else:
            rospy.logwarn("Result file not found.")

    except CvBridgeError as e:
        rospy.logerr(f"Failed to convert ROS image to OpenCV format: {e}")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
    finally:
        rospy.signal_shutdown("Image processed")

def bottle_detection_ready_callback(msg):
    if msg.data == "Ready to detect the bottle.":
        rospy.loginfo("Received message: Ready to detect the bottle.")
        # Subscribe to the RGB-D camera image topic
        camera = rospy.get_param('~camera', '/cam_2')
        image_sub = rospy.Subscriber(camera + "/color/image_raw", Image, handle_single_image)

# Subscribe to the bottle detection readiness topic
ready_sub = rospy.Subscriber("bottle_detection_ready", String, bottle_detection_ready_callback)

# Keep the script running
rospy.spin()
