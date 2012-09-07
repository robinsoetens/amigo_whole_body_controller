#include "WholeBodyController.h"

WholeBodyController::WholeBodyController() {

    initialize();

}

WholeBodyController::~WholeBodyController() {

    ROS_INFO("Shutting down whole body controller");

}

bool WholeBodyController::initialize() {

    // Get node handle
    ros::NodeHandle n("~");
    ROS_INFO("Initializing whole body controller");

    // ToDo: standardize data types joint positions
    // ToDo: fill in odom topic
    //odom_sub_ = ;
    measured_torso_position_sub_ = n.subscribe<std_msgs::Float64>("/spindle_position", 10, &WholeBodyController::callbackMeasuredTorsoPosition, this);
    measured_left_arm_position_sub_ = n.subscribe<amigo_msgs::arm_joints>("/arm_left_controller/joint_measurements", 10, &WholeBodyController::callbackMeasuredLeftArmPosition, this);
    measured_right_arm_position_sub_ = n.subscribe<amigo_msgs::arm_joints>("/arm_right_controller/joint_measurements", 10, &WholeBodyController::callbackMeasuredRightArmPosition, this);
    measured_head_pan_sub_ = n.subscribe<std_msgs::Float64>("/head_pan_angle", 10, &WholeBodyController::callbackMeasuredHeadPan, this);
    measured_head_tilt_sub_ = n.subscribe<std_msgs::Float64>("/head_tilt_angle", 10, &WholeBodyController::callbackMeasuredHeadTilt, this);

    return true;

}

void WholeBodyController::callbackMeasuredTorsoPosition(const std_msgs::Float64::ConstPtr& msg) {

}

void WholeBodyController::callbackMeasuredLeftArmPosition(const amigo_msgs::arm_joints::ConstPtr& msg) {

}

void WholeBodyController::callbackMeasuredRightArmPosition(const amigo_msgs::arm_joints::ConstPtr& msg) {

}

void WholeBodyController::callbackMeasuredHeadPan(const std_msgs::Float64::ConstPtr& msg) {

}

void WholeBodyController::callbackMeasuredHeadTilt(const std_msgs::Float64::ConstPtr& msg) {

}


