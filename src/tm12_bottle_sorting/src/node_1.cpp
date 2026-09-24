// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tm_msgs/SetPositions.h>
#include <tm_msgs/FeedbackState.h>

// std headers
#include <sstream>
#include <cstdlib>
#include <cmath>

// Global variables to store feedback states
std::vector<double> current_joint_positions(6, 0.0);
bool position_reached = false;
const double tolerance = 0.01; // 1% of the target joint angle

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "demo_set_positions");      
    ros::NodeHandle nh_demo; 
    ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
    ros::Publisher feedback_pub = nh_demo.advertise<std_msgs::String>("robot_feedback", 10);
    ros::Subscriber feedback_sub = nh_demo.subscribe("/feedback_states", 1, feedbackCallback);

    tm_msgs::SetPositions srv;
    
    // Target positions in radians
    std::vector<double> target_positions = {0.3287, -0.2046, 1.7309, 0.0108, 1.5445, 0.2288};

    // Request
    srv.request.motion_type = tm_msgs::SetPositions::Request::PTP_J;
    srv.request.positions = target_positions;
    srv.request.velocity = 20; // rad/s
    srv.request.acc_time = 0.2;
    srv.request.blend_percentage = 10;
    srv.request.fine_goal = false;

    if (client.call(srv)) {
        if (srv.response.ok) {
            ROS_INFO_STREAM("SetPositions to robot");
        } else {
            ROS_WARN_STREAM("SetPositions to robot, but response not yet ok");
        }
    } else {
        ROS_ERROR_STREAM("Error SetPositions to robot");
        return 1;
    }

    // Check if the robot has reached the position
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();
        
        if (isPositionReached(target_positions, current_joint_positions)) {
            std_msgs::String msg;
            msg.data = "Robot has reached the position to detect the bottle";
            feedback_pub.publish(msg);
            ROS_INFO_STREAM("Robot has reached the position to detect the bottle");
            break;
        }

        loop_rate.sleep();
    }

    return 0;
}
