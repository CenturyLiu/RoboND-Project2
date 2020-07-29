#include "ros/ros.h"
#include "ball_chaser/ArmToTarget.h"
#include <std_msgs/Float64.h>

// Global joint publisher variables
ros::Publisher left_arm_pub, right_arm_pub;

// This function checks and clamps the joint angles to a safe zone
std::vector<float> clamp_at_boundaries(float requested_j1, float requested_j2){
    // Define clamped joint angles and assign then tothe request ones
    float clamped_j1 = requested_j1;
    float clamped_j2 = requested_j2;

    // Get min and max joint parameters, and assigning them to their respective variables
    float min_j1, max_j1, min_j2, max_j2;
    //Assign a new node handle since we have no access to the main one
    ros::NodeHandle n2;

    //Get node name
    std::string node_name = ros::this_node::getName();
    // Get joints min and max parameters
    n2.getParam(node_name + "/min_left_arm_angle",min_j1);
    n2.getParam(node_name + "/max_left_arm_angle",max_j1);
    n2.getParam(node_name + "/min_right_arm_angle",min_j2);
    n2.getParam(node_name + "/max_right_arm_angle",max_j2);

    // check if joint 1 falls in the safe zone, otherwise clamp it
    if (requested_j1 < min_j1 || requested_j1 > max_j1){
        clamped_j1 = std::min(std::max(requested_j1, min_j1),max_j1);
        ROS_WARN("left arm angle is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j1, max_j1, clamped_j1);
    }

    // check if joint 2 falls in the safe zone, otherwise clamp it
    if (requested_j2 < min_j2 || requested_j2 > max_j2){
        clamped_j2 = std::min(std::max(requested_j2, min_j2),max_j2);
        ROS_WARN("right arm angle is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j2, max_j2, clamped_j2);
    }

    // Store clamped joint angles in a clamed_data vector
    std::vector<float> clamped_data = { clamped_j1, clamped_j2};

    return clamped_data;
}

// This callback function executes whenever a safe_move service is requested
bool handle_safe_move_request(ball_chaser::ArmToTarget::Request& req, ball_chaser::ArmToTarget::Response& res ){

    ROS_INFO("ArmToTarget Request received - j1:%1.2f, j2:%1.2f", (float)req.left_arm, (float)req.right_arm);

    // Check if requested joint angles are in the safe zone, otherwise clamp them
    std::vector<float> joint_angles = clamp_at_boundaries(req.left_arm,req.right_arm);

    // Publish clamped joint angles to the arm
    std_msgs::Float64 left_angle, right_angle;

    left_angle.data = joint_angles[0];
    right_angle.data = joint_angles[1];

    left_arm_pub.publish(left_angle);
    right_arm_pub.publish(right_angle);

    // Wait 3 seconds for arm to settle
    ros::Duration(3).sleep();

    // Return a response message
    res.msg_feedback = "Arm angles set - left: " + std::to_string(joint_angles[0]) + " , right: " + std::to_string(joint_angles[1]);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;

}

int main(int argc, char** argv){

    // Initialize the arm_mover node and create a handle to it
    ros::init(argc, argv, "move_arm");
    ros::NodeHandle n;

    // Define two publishers to publish std_msgs::Float64 messages on joints respective topics
    left_arm_pub = n.advertise<std_msgs::Float64>("/my_robot/left_arm_joint_position_controller/command", 10);
    right_arm_pub = n.advertise<std_msgs::Float64>("/my_robot/right_arm_joint_position_controller/command", 10);

    // Define a safe_mover service with a handle_safe_move_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/move_arm", handle_safe_move_request);
    ROS_INFO("Ready to send arm angle commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}