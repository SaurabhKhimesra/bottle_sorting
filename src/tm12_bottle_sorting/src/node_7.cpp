#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <boost/filesystem.hpp>
#include "geometry_msgs/Pose.h"
#include "tm_msgs/SetPositions.h"
#include "tm_msgs/FeedbackState.h"
#include <std_msgs/String.h>

// Include the Boost filesystem namespace
namespace fs = boost::filesystem;

// Global variables to store feedback states
std::vector<double> current_joint_positions(6, 0.0);
bool position_reached = false;
const double tolerance = 0.02; // 2% tolerance

struct Detection {
    int class_id;
    float x_center, y_center, width, height;
};

void feedbackCallback(const tm_msgs::FeedbackState::ConstPtr& msg) {
    current_joint_positions = msg->joint_pos;
}

// Function to check if the current position is within the tolerance range of the target position
bool isPositionReached(const std::vector<double>& target_positions, const std::vector<double>& current_positions) {
    for (size_t i = 0; i < target_positions.size(); ++i) {
        double diff = std::abs(current_positions[i] - target_positions[i]);
        if (diff > tolerance * std::abs(target_positions[i])) {
            return false;
        }
    }
    return true;
}

std::vector<Detection> read_detections(const std::string& file_path) {
    std::vector<Detection> detections;
    std::ifstream file(file_path);
    if (!file.is_open()) {
        ROS_ERROR_STREAM("Unable to open file: " << file_path);
        return detections;
    }
    std::string line;
    while (getline(file, line)) {
        std::istringstream iss(line);
        Detection det;
        if (iss >> det.class_id >> det.x_center >> det.y_center >> det.width >> det.height) {
            detections.push_back(det);
        }
    }
    file.close();
    return detections;
}

bool move_robot_to_position(ros::ServiceClient& client, const std::vector<double>& positions) {
    tm_msgs::SetPositions srv;
    srv.request.motion_type = tm_msgs::SetPositions::Request::PTP_J;
    srv.request.positions = positions;
    srv.request.velocity = 10;  // rad/s
    srv.request.acc_time = 1;
    srv.request.blend_percentage = 10;
    srv.request.fine_goal = false;

    if (client.call(srv)) {
        if (srv.response.ok) {
            ROS_INFO("SetPositions to robot");
            return true;
        } else {
            ROS_WARN("SetPositions to robot, but response not yet ok");
            return false;
        }
    } else {
        ROS_ERROR("Error SetPositions to robot");
        return false;
    }
}

void bottle_detection_ready_callback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "Brand of the bottle has been detected") {
        ROS_INFO("Received message: Brand of the bottle has been detected");

        ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
        ros::Publisher feedback_pub = nh.advertise<std_msgs::String>("robot_feedback", 10);
        ros::Subscriber feedback_sub = nh.subscribe("/feedback_states", 1, feedbackCallback);

        std::map<int, std::vector<double>> initial_positions = {
        {0, {-1.8775, 0.1821, 1.1813, 0.1623, 1.5616, -0.2127}},
        {1, {-1.18, -0.20, 1.70, -0.04, 1.52, -0.04}},
        {2, {-0.76, 0.08, 1.53, -0.07, 1.53, 0.36}}
    };

    std::map<int, std::vector<std::vector<double>>> sub_positions = {
        {0, {
            {-1.8313, 0.0836, 1.7618, -0.3749, 1.5569, -0.3558},  // Sub-position 1
            {1.7296, 0.0455, 1.7919, -0.3317, 1.5691, -0.3557},  // Sub-position 2
            {-1.6796, 0.1251, 1.7056, -0.3676, 1.5105, -0.2313}   // Sub-position 3
        }},
        {1, {
            {-1.18, -0.04, 1.90, -0.34, 1.52, -0.04},  // Sub-position 1
            {-1.18, -0.04, 1.90, -0.34, 1.52, -0.04},  // Sub-position 2
            {-1.18, -0.04, 1.90, -0.34, 1.52, -0.04}   // Sub-position 3
        }},
        {2, {
            {-0.76, 0.20, 1.57, -0.21, 1.55, 0.36},  // Sub-position 1
            {-0.69, 0.33, 1.38, -0.15, 1.56, 0.45},  // Sub-position 2
            {-0.7565, 0.3345, 1.3953, -0.2230, 1.5043, 0.0378}   // Sub-position 3
        }}
    };

        // yolov5 is a sibling of this package in the workspace; the counters
        // live beside the frames the vision nodes write.
        const std::string package_path = ros::package::getPath("tm12_bottle_sorting");

        std::string runtime_dir, yolov5_dir;
        ros::param::param<std::string>("~runtime_dir", runtime_dir, package_path + "/runtime");
        ros::param::param<std::string>("~yolov5_dir", yolov5_dir, package_path + "/../yolov5");

        const std::string base_directory = yolov5_dir + "/runs/detect";
        if (!fs::exists(base_directory)) {
            ROS_ERROR_STREAM("YOLOv5 output directory not found: " << base_directory
                             << " (set the yolov5_dir parameter)");
            return;
        }

        fs::path latest_dir;
        std::time_t latest_time = 0;

        for (const auto& entry : fs::directory_iterator(base_directory)) {
            if (fs::is_directory(entry) && entry.path().filename().string().find("static_output") != std::string::npos) {
                std::time_t time = fs::last_write_time(entry);
                if (time > latest_time) {
                    latest_dir = entry;
                    latest_time = time;
                }
            }
        }

        if (latest_dir.empty()) {
            ROS_ERROR("No output directory found");
            return;
        }

        fs::path labels_directory = latest_dir / "labels";
        fs::path latest_file;
        latest_time = 0;

        for (const auto& entry : fs::directory_iterator(labels_directory)) {
            if (fs::is_regular_file(entry) && entry.path().filename().string().find("camera_image_256.txt") != std::string::npos) {
                std::time_t time = fs::last_write_time(entry);
                if (time > latest_time) {
                    latest_file = entry;
                    latest_time = time;
                }
            }
        }

        if (latest_file.empty()) {
            ROS_ERROR("No detection file found in the latest directory");
            return;
        }

        std::vector<Detection> detections = read_detections(latest_file.string());

        for (const auto& det : detections) {
            int class_id = det.class_id;
            if (initial_positions.count(class_id) > 0) {
                // Move to the initial position
                if (!move_robot_to_position(client, initial_positions[class_id])) {
                    ROS_ERROR("Failed to move robot to initial position for class ID %d", class_id);
                    return;
                }

                // Check if the robot has reached the initial position
                ros::Rate loop_rate(10); // 10 Hz
                while (ros::ok()) {
                    ros::spinOnce();
                    
                    if (isPositionReached(initial_positions[class_id], current_joint_positions)) {
                        std_msgs::String msg;
                        msg.data = "Robot has reached the initial position for class ID " + std::to_string(class_id);
                        feedback_pub.publish(msg);
                        ROS_INFO_STREAM("Robot has reached the initial position for class ID " << class_id);
                        break;
                    }

                    loop_rate.sleep();
                }

                // Define paths for count and reset files
                fs::create_directories(runtime_dir);
                std::string count_file = runtime_dir + "/detection_count_" + std::to_string(class_id) + ".txt";
                std::string reset_file = runtime_dir + "/reset_" + std::to_string(class_id) + ".txt";

                // Load the count from file or set to 0 if not present
                int count = 0;
                std::ifstream infile(count_file);
                if (infile.is_open()) {
                    infile >> count;
                    infile.close();
                }

                // Determine sub-position based on the count
                int sub_position_index = count % 3;
                const std::vector<double>& sub_position = sub_positions[class_id][sub_position_index];

                // Move to the sub-position
                if (!move_robot_to_position(client, sub_position)) {
                    ROS_ERROR("Failed to move robot to sub-position %d for class ID %d", sub_position_index, class_id);
                    return;
                }

                // Check if the robot has reached the sub-position
                while (ros::ok()) {
                    ros::spinOnce();
                    
                    if (isPositionReached(sub_position, current_joint_positions)) {
                        std_msgs::String msg;
                        msg.data = "Ready to place the branded bottle for class ID " + std::to_string(class_id);
                        feedback_pub.publish(msg);
                        ROS_INFO_STREAM("Ready to place the branded bottle for class ID " << class_id);
                        break;
                    }

                    loop_rate.sleep();
                }

                // Check if maximum sub-position is reached and alert
                if (sub_position_index == 2) {
                    ROS_WARN("Maximum sub-position reached for class ID %d. Manual reset required.", class_id);
                }

                // Increment and save the count
                count++;
                std::ofstream outfile(count_file);
                if (outfile.is_open()) {
                    outfile << count;
                    outfile.close();
                }

                // Manual reset logic (set count to 0 manually in the file)
                std::ifstream reset_infile(reset_file);
                int reset = 1;
                if (reset_infile.is_open()) {
                    reset_infile >> reset;
                    reset_infile.close();
                    if (reset == 0) {
                        count = 0;
                        std::ofstream reset_outfile(count_file);
                        if (reset_outfile.is_open()) {
                            reset_outfile << count;
                            reset_outfile.close();
                        }
                        ROS_INFO("Count for class ID %d has been reset to 0.", class_id);
                        std::ofstream reset_flag_outfile(reset_file);
                        if (reset_flag_outfile.is_open()) {
                            reset_flag_outfile << 1;
                            reset_flag_outfile.close();
                        }
                    }
                }
            } else {
                ROS_WARN("No initial position defined for class ID %d", class_id);
            }
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "demo_set_positions");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("detection_message_topic", 10, bottle_detection_ready_callback);

    ros::spin();

    return 0;
}
