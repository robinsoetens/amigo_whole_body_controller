#include "WholeBodyController.h"

WholeBodyController::WholeBodyController() {

    initialize();

}

WholeBodyController::~WholeBodyController() {

    ROS_INFO("Shutting down whole body controller");
    measured_torso_position_sub_.shutdown();
    measured_left_arm_position_sub_.shutdown();
    measured_right_arm_position_sub_.shutdown();
    measured_head_pan_sub_.shutdown();
    measured_head_tilt_sub_.shutdown();

}

bool WholeBodyController::initialize() {

    // Get node handle
    ros::NodeHandle n("~");
    //ROS_INFO("Nodehandle %s",n.getNamespace().c_str());
    ROS_INFO("Initializing whole body controller");

    // ToDo: standardize data types joint positions
    // ToDo: fill in odom topic
    //odom_sub_ = ;
    measured_torso_position_sub_ = n.subscribe<std_msgs::Float64>("/spindle_position", 10, &WholeBodyController::callbackMeasuredTorsoPosition, this);
    measured_left_arm_position_sub_ = n.subscribe<amigo_msgs::arm_joints>("/arm_left_controller/joint_measurements", 10, &WholeBodyController::callbackMeasuredLeftArmPosition, this);
    measured_right_arm_position_sub_ = n.subscribe<amigo_msgs::arm_joints>("/arm_right_controller/joint_measurements", 10, &WholeBodyController::callbackMeasuredRightArmPosition, this);
    measured_head_pan_sub_ = n.subscribe<std_msgs::Float64>("/head_pan_angle", 10, &WholeBodyController::callbackMeasuredHeadPan, this);
    measured_head_tilt_sub_ = n.subscribe<std_msgs::Float64>("/head_tilt_angle", 10, &WholeBodyController::callbackMeasuredHeadTilt, this);

    // Implement Jacobian matrix
    //ComputeJacobian_.initialize();

    // Implement left Cartesian Impedance
    CIleft_.initialize();

    ROS_INFO("Whole Body Controller Initialized");

    return true;

}

bool WholeBodyController::update() {

    //ROS_INFO("Updating wholebodycontroller");
    //ComputeJacobian_.getJacobian(q_current_);

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

//TEMP
/*
void WholeBodyController::callbackTarget(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    ROS_INFO("Received target");
    //Transform message into tooltip frame. This directly implies e = x_d - x
    errorPose = WholeBodyController::transformPose(listener, *msg);
    ROS_DEBUG("errorpose = %f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%f",errorPose.pose.position.x,errorPose.pose.position.y,errorPose.pose.position.z,errorPose.pose.orientation.x,errorPose.pose.orientation.y,errorPose.pose.orientation.z,errorPose.pose.orientation.w);

}

//TEMP
geometry_msgs::PoseStamped WholeBodyController::transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped poseMsg){

    geometry_msgs::PoseStamped poseInGripperFrame;

    listener.transformPose("/grippoint_left",poseMsg,poseInGripperFrame);

    return poseInGripperFrame;

}*/
