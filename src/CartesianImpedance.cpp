#include "CartesianImpedance.h"

CartesianImpedance::CartesianImpedance() {

}

CartesianImpedance::~CartesianImpedance() {

}

bool CartesianImpedance::initialize() {

    ROS_INFO("Initializing left Cartesian Impedance");

    // Get node handle
    ros::NodeHandle n("~");
    //ROS_INFO("Nodehandle %s",n.getNamespace().c_str());

    target_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/arm_left_target_pose", 10, &CartesianImpedance::callbackTarget, this);
    ROS_INFO("Subscribed to %s topic for target",target_sub_.getTopic().c_str());

    ROS_INFO("Initialized left Cartesian Impedance");

    return true;

}

void CartesianImpedance::callbackTarget(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    ROS_INFO("Received target");
    //Transform message into tooltip frame. This directly implies e = x_d - x
    errorPose = CartesianImpedance::transformPose(listener, *msg);
    ROS_DEBUG("errorpose = %f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%f",errorPose.pose.position.x,errorPose.pose.position.y,errorPose.pose.position.z,errorPose.pose.orientation.x,errorPose.pose.orientation.y,errorPose.pose.orientation.z,errorPose.pose.orientation.w);

}

geometry_msgs::PoseStamped CartesianImpedance::transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped poseMsg){

    geometry_msgs::PoseStamped poseInGripperFrame;

    listener.transformPose("/grippoint_left",poseMsg,poseInGripperFrame);

    return poseInGripperFrame;

}
