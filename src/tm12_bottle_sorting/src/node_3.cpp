// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <fstream> // Include for file I/O
#include <vector>
#include <tm_msgs/SetPositions.h>
#include <tm_msgs/FeedbackState.h>
#include <cmath>

// Global variables to store feedback states
std::vector<double> current_joint_positions(6, 0.0);
bool position_reached = false;
const double tolerance = 0.01; // 1% tolerance

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

// Define a function to return the desired position based on an index
std::vector<double> getPosition(int index) {
    // Define all 16 positions as a 2D vector
    std::vector<std::vector<double>> positions = {
        {4.389, -1.140, 123.551, -31.306, 89.372, 10.688},
        {4.389, -1.140, 123.551, -31.306, 89.372, 10.688},
        {12.785, -3.098, 125.103, -32.152, 89.421, 11.272},
        {21.120, -2.819, 124.258, -30.352, 88.117, 19.316},
        {28.316, -1.111, 124.261, -31.876, 88.255, 26.508},
        {4.583, 5.505, 118.005, -32.911, 87.904, 2.772},
        {12.098, 4.405, 118.116, -31.693, 87.984, 10.293},
        {18.556, 4.752, 117.601, -31.336, 88.075, 16.755},
        {24.471, 5.985, 117.050, -31.859, 88.178, 22.670},
        {3.786, 11.868, 109.824, -31.115, 87.891, 1.988},
        {10.518, 11.584, 110.555, -31.354, 87.959, 8.721},
        {16.774, 11.244, 109.788, -30.065, 88.041, 14.984},
        {22.099, 12.762, 108.712, -30.360, 88.128, 20.310},
        {3.954, 19.007, 101.170, -29.594, 87.884, 2.169},
        {9.441, 18.620, 101.617, -28.488, 87.939, 7.659},
        {14.791, 18.869, 101.121, -29.081, 88.006, 13.013},
        {20.104, 19.705, 100.256, -28.903, 88.088, 18.329}
    };

    // Ensure the index is within the valid range
    if (index >= 0 && index < positions.size()) {
        // Convert degrees to radians for each joint position
        for (auto &pos : positions[index]) {
            pos *= 0.0174533; // Conversion factor from degrees to radians
        }
        return positions[index];
    } else {
        ROS_ERROR_STREAM("Invalid position index");
        return std::vector<double>(); // Return an empty vector in case of an invalid index
    }
}

void gripperFeedbackCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "sending co-ordinates of the detected bottle to the gripper") {
        position_reached = true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "demo_set_positions");
    ros::NodeHandle nh_demo;
    ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
    ros::Publisher feedback_pub = nh_demo.advertise<std_msgs::String>("robot_feedback", 10);
    ros::Subscriber feedback_sub = nh_demo.subscribe("/feedback_states", 1, feedbackCallback);
    ros::Subscriber gripper_feedback_sub = nh_demo.subscribe("gripper_feedback", 1, gripperFeedbackCallback);

    tm_msgs::SetPositions srv;

    // Wait for the message from the gripper
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok() && !position_reached) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Read the position index written by bottle_position.py. Same default as
    // the Python side: runtime/ inside the package, overridable per launch.
    std::string runtime_dir;
    ros::param::param<std::string>("~runtime_dir", runtime_dir,
                                   ros::package::getPath("tm12_bottle_sorting") + "/runtime");

    const std::string position_file = runtime_dir + "/min_bottle_position.txt";
    std::ifstream inputFile(position_file);
    int positionIndex;
    if (inputFile >> positionIndex) {
        ROS_INFO_STREAM("Read minimum position index from file: " << positionIndex);
    } else {
        ROS_ERROR_STREAM("Failed to read position index from " << position_file);
        return 1; // Exit if unable to read file
    }
    inputFile.close();

    // Retrieve the desired position based on the file input
    std::vector<double> jointPositions = getPosition(positionIndex);

    // Set up the request
    srv.request.motion_type = tm_msgs::SetPositions::Request::PTP_J;
    for (double pos : jointPositions) {
        srv.request.positions.push_back(pos);
    }
    srv.request.velocity = 0.9; // rad/s
    srv.request.acc_time = 0.2;
    srv.request.blend_percentage = 10;
    srv.request.fine_goal = false;

    // Call the service
    if (client.call(srv)) {
        if (srv.response.ok) {
            ROS_INFO_STREAM("SetPositions to robot");
        } else {
            ROS_WARN_STREAM("SetPositions to robot, but response not yet ok");
        }
    } else {
        ROS_ERROR_STREAM("Error setting positions to robot");
        return 1;
    }

    // Check if the robot has reached the position
    while (ros::ok()) {
        ros::spinOnce();

        if (isPositionReached(jointPositions, current_joint_positions)) {
            std_msgs::String msg;
            msg.data = "Robot is ready to grip";
            feedback_pub.publish(msg);
            ROS_INFO_STREAM("Robot is ready to grip");
            break;
        }

        loop_rate.sleep();
    }

    return 0;
}
