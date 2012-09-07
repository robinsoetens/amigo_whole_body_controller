#include "CartesianImpedance.h"

CartesianImpedance::CartesianImpedance() {

}

CartesianImpedance::~CartesianImpedance() {

}

bool CartesianImpedance::initialize() {

    // Get node handle
    ros::NodeHandle n("~");

    target_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/arm_left_target_pose", 10, &CartesianImpedance::callbackTarget, this);

    return true;

}

void CartesianImpedance::callbackTarget(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    //Transform message into tooltip frame. This directly implies e = x_d - x
    errorPose = CartesianImpedance::transformPose(listener, *msg);

}

geometry_msgs::PoseStamped CartesianImpedance::transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped poseMsg){

    geometry_msgs::PoseStamped poseInGripperFrame;

    listener.transformPose("/grippoint_left",poseMsg,poseInGripperFrame);

    return poseInGripperFrame;

}
