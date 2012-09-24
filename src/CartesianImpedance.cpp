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

    // Fill in impedance matrix
    // ToDo: Parametrize
    K_.resize(6,6);
    double standard_factor = 1;
    for (uint i = 0; i<6; i++) {
        for (uint ii = 0; ii<6; ii++) {
            if (i != ii) K_(i,ii) = 0;
            else K_(i,ii) = standard_factor;
        }
    }
    error_vector_.resize(6);
    for (uint i = 0; i<6; i++) error_vector_(i) = 0;

    ROS_INFO("Initialized left Cartesian Impedance");

    return true;

}

void CartesianImpedance::callbackTarget(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    ROS_INFO("Received target");
    //Transform message into tooltip frame. This directly implies e = x_d - x
    errorPose = CartesianImpedance::transformPose(listener, *msg);
    ROS_DEBUG("errorpose = %f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%f",errorPose.pose.position.x,errorPose.pose.position.y,errorPose.pose.position.z,errorPose.pose.orientation.x,errorPose.pose.orientation.y,errorPose.pose.orientation.z,errorPose.pose.orientation.w);

    // ToDo: Fill in error vector orientations as well!!!
    error_vector_(0) = errorPose.pose.position.x;
    error_vector_(1) = errorPose.pose.position.y;
    error_vector_(2) = errorPose.pose.position.z;
    error_vector_(3) = 0;
    error_vector_(4) = 0;
    error_vector_(5) = 0;

}

geometry_msgs::PoseStamped CartesianImpedance::transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped poseMsg){

    geometry_msgs::PoseStamped poseInGripperFrame;

    // Wait for transform???
    listener.transformPose("/grippoint_left",poseMsg,poseInGripperFrame);

    return poseInGripperFrame;

}

void CartesianImpedance::update(Eigen::MatrixXd Jacobian, Eigen::VectorXd& tau) {

    ///ROS_INFO("Updating Cartesian Impedance");

    // ToDo: Make this variable
    Eigen::MatrixXd sub_jacobian = Jacobian.block(0,0,6,Jacobian.cols());

    tau += sub_jacobian.transpose() * K_ * error_vector_;

    ///ROS_INFO("tau %f %f %f %f", tau(0),  tau(1),  tau(2),  tau(3));
    ///ROS_INFO("FSpindle %f", tau(7));

}
