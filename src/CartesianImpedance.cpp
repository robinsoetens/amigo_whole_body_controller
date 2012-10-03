#include "CartesianImpedance.h"
#include <tf/transform_datatypes.h>

CartesianImpedance::CartesianImpedance() {

}

CartesianImpedance::~CartesianImpedance() {

}

bool CartesianImpedance::initialize(std::string end_effector_frame) {

    ROS_INFO("Initializing Cartesian Impedance");

    // Get node handle
    ros::NodeHandle n("~");
    std::string ns = ros::this_node::getName();
    //ROS_INFO("Nodehandle %s",n.getNamespace().c_str());

    end_effector_frame_ = end_effector_frame;

    if (end_effector_frame_ == "/grippoint_left") {
        target_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/arm_left_target_pose", 10, &CartesianImpedance::callbackTarget, this);
    }
    else if (end_effector_frame_ == "/grippoint_right") {
        target_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/arm_right_target_pose", 10, &CartesianImpedance::callbackTarget, this);
    }
    ROS_INFO("Subscribed to %s topic for target",target_sub_.getTopic().c_str());

    // Fill in impedance matrix
    // ToDo: Parametrize
    K_.resize(6,6);
    //double standard_factor = 25;
    for (uint i = 0; i<6; i++) {
        for (uint ii = 0; ii<6; ii++) {
            K_(i,ii) = 0;
            ///if (i != ii) K_(i,ii) = 0;
            ///else K_(i,ii) = standard_factor;
        }
    }
    n.param<double> (ns+"/kx", K_(0,0), 26);
    n.param<double> (ns+"/ky", K_(1,1), 26);
    n.param<double> (ns+"/kz", K_(2,2), 26);
    n.param<double> (ns+"/krx", K_(3,3), 26);
    n.param<double> (ns+"/kry", K_(4,4), 26);
    n.param<double> (ns+"/krz", K_(5,5), 26);
    ///ROS_INFO("K = %f, %f, %f, %f, %f, %f",K_(0,0),K_(1,1),K_(2,2),K_(3,3),K_(4,4),K_(5,5));

    error_vector_.resize(6);
    for (uint i = 0; i<6; i++) error_vector_(i) = 0;

    // Wait for transform
    //ToDo: make this work!
    bool wait_for_transform = listener.waitForTransform(end_effector_frame_,"/map", ros::Time::now(), ros::Duration(1.0));
    if (!wait_for_transform) ROS_WARN("Transform between %s and /map is not available",end_effector_frame.c_str());

    ROS_INFO("Initialized Cartesian Impedance");

    return true;

}

void CartesianImpedance::callbackTarget(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    ROS_INFO("Received target");
    //Transform message into tooltip frame. This directly implies e = x_d - x
    errorPose = CartesianImpedance::transformPose(listener, *msg);

    // ToDo: Check orientations as well
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(errorPose.pose.orientation, orientation);
    double roll, pitch, yaw;
    btMatrix3x3(orientation).getRPY(roll, pitch, yaw);
    error_vector_(0) = errorPose.pose.position.x;
    error_vector_(1) = errorPose.pose.position.y;
    error_vector_(2) = errorPose.pose.position.z;
    error_vector_(3) = roll;
    error_vector_(4) = pitch;
    error_vector_(5) = yaw;

    ROS_DEBUG("errorpose = %f,\t%f,\t%f,\t%f,\t%f,\t%f",error_vector_(0),error_vector_(1),error_vector_(2),error_vector_(3),error_vector_(4),error_vector_(5));

}

geometry_msgs::PoseStamped CartesianImpedance::transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped poseMsg){

    geometry_msgs::PoseStamped poseInGripperFrame;

    // Wait for transform???
    listener.transformPose(end_effector_frame_,poseMsg,poseInGripperFrame);

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
