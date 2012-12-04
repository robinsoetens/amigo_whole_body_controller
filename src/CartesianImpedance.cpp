#include "CartesianImpedance.h"
#include <tf/transform_datatypes.h>

#include <amigo_whole_body_controller/ArmTaskAction.h>
#include <actionlib/server/simple_action_server.h>

CartesianImpedance::CartesianImpedance(const std::string& end_effector_frame, tf::TransformListener* tf_listener) :
    is_active_(false), chain_(0), end_effector_frame_(end_effector_frame), tf_listener_(tf_listener) {

    ROS_INFO("Initializing Cartesian Impedance for %s", end_effector_frame_.c_str());

    // Get node handle
    ros::NodeHandle n("~");
    std::string ns = ros::this_node::getName();

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
    bool wait_for_transform = tf_listener_->waitForTransform(end_effector_frame_,"/map", ros::Time::now(), ros::Duration(1.0));
    if (!wait_for_transform) ROS_WARN("Transform between %s and /map is not available",end_effector_frame.c_str());

    // Pre-grasping
    pre_grasp_ = false;
    n.param<double> (ns+"/pre_grasp_delta", pre_grasp_delta_, 0.0);
    /////
    ROS_WARN("force_vector_index in update hook is now obsolete");

    ROS_INFO("Initialized Cartesian Impedance");
}

bool CartesianImpedance::initialize(const std::vector<Chain*>& chains) {
    chain_ = 0;
    for(std::vector<Chain*>::const_iterator it_chain = chains.begin(); it_chain != chains.end(); ++it_chain) {
        Chain* chain = *it_chain;
        if (chain->hasLink(end_effector_frame_)) {
            chain_ = chain;
        }
    }

    return (chain_ != 0);
}

bool CartesianImpedance::isActive() {
    return is_active_;
}

CartesianImpedance::~CartesianImpedance() {
}

void CartesianImpedance::setGoal(tf::Stamped<tf::Pose>& goal_pose) {
    goal_pose_ = goal_pose;
    is_active_ = true;
}

void CartesianImpedance::apply() {

    ROS_INFO("CartesianImpedance::apply");

    Eigen::VectorXd F_task(6);

    if (!is_active_) {
        return;
    }

    ROS_INFO("    is active");

    //Transform message into tooltip frame. This directly implies e = x_d - x
    /////errorPose = CartesianImpedance::transformPose(listener, goal_pose_);
    //errorPose = CartesianImpedance::transformPose(goal_pose_);

    //listener.transformPose(end_effector_frame_,goal_pose_,errorPose);

    // ToDo: why doesn't the state above work? Seems more sensible than below in terms of realtime behavior

    //listener.waitForTransform()

    //ros::Time now = ros::Time::now();
    //bool wait_for_transform = listener.waitForTransform(end_effector_frame_,goal_pose_.header.frame_id, now, ros::Duration(1.0));

    goal_pose_.stamp_ = ros::Time();
    try {
        tf_listener_->transformPose("/grippoint_left", goal_pose_, error_pose_); // end_effector_frame_
    } catch (tf::TransformException& e) {
        ROS_ERROR("CartesianImpedance: %s", e.what());
        return;
    }

    //listener.transformPose(end_effector_frame_,now,goal_pose_,goal_pose_.header.frame_id,errorPose);

    // ToDo: Check orientations as well
    double roll, pitch, yaw;

#if ROS_VERSION_MINIMUM(1, 8, 0)
    tf::Matrix3x3(error_pose_.getRotation()).getRPY(roll, pitch, yaw);
#else
    btMatrix3x3(error_pose_.getRotation()).getRPY(roll, pitch, yaw);
#endif

    error_vector_(0) = error_pose_.getOrigin().getX();
    error_vector_(1) = error_pose_.getOrigin().getY();
    error_vector_(2) = error_pose_.getOrigin().getZ();
    error_vector_(3) = roll;
    error_vector_(4) = pitch;
    error_vector_(5) = yaw;

    std::cout << "goal_pose = " << goal_pose_.getOrigin().getX() << " , " << goal_pose_.getOrigin().getY() << " , " << goal_pose_.getOrigin().getZ() << std::endl;

    std::cout << "error_vector = " << error_vector_ << std::endl;

    // If pre-grasping, an offset is added to the errorpose in x-direction
    //if (pre_grasp_) error_vector_(0) -= pre_grasp_delta_;

    ///ROS_INFO("errorpose = %f,\t%f,\t%f,\t%f,\t%f,\t%f",error_vector_(0),error_vector_(1),error_vector_(2),error_vector_(3),error_vector_(4),error_vector_(5));

    F_task = K_ * error_vector_;

    // add the wrench to the end effector of the kinematic chain
    chain_->addCartesianWrench(end_effector_frame_, F_task);
}
