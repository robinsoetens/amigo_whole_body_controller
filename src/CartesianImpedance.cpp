#include "CartesianImpedance.h"
#include <tf/transform_datatypes.h>

#include <amigo_whole_body_controller/ArmTaskAction.h>
#include <actionlib/server/simple_action_server.h>

CartesianImpedance::CartesianImpedance(const std::string& end_effector_frame) :
    is_active_(false), chain_(0), end_effector_frame_(end_effector_frame) {

    ROS_INFO("Initializing Cartesian Impedance for %s", end_effector_frame_.c_str());

    // ToDo: Parameterize
    Ts = 0.1;

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
    //ROS_INFO("K = %f, %f, %f, %f, %f, %f",K_(0,0),K_(1,1),K_(2,2),K_(3,3),K_(4,4),K_(5,5));

    error_vector_.resize(6);
    for (uint i = 0; i<6; i++) error_vector_(i) = 0;

    // Wait for transform
    //ToDo: make this work!
    //bool wait_for_transform = tf_listener_->waitForTransform(end_effector_frame_,"/map", ros::Time::now(), ros::Duration(1.0));
    //if (!wait_for_transform) ROS_WARN("Transform between %s and /map is not available",end_effector_frame.c_str());

    // Pre-grasping
    pre_grasp_ = false;
    n.param<double> (ns+"/pre_grasp_delta", pre_grasp_delta_, 0.0);
    /////
    ROS_WARN("force_vector_index in update hook is now obsolete");

    ROS_INFO("Initialized Cartesian Impedance");
}

bool CartesianImpedance::initialize() {
    return true;
}

bool CartesianImpedance::isActive() {
    return is_active_;
}

CartesianImpedance::~CartesianImpedance() {
}

void CartesianImpedance::setGoal(geometry_msgs::PoseStamped& goal_pose) {
    goal_pose_ = goal_pose;
    is_active_ = true;
    status_ = 2;
}

void CartesianImpedance::cancelGoal() {
    is_active_ = false;
    status_ = 0;
}

void CartesianImpedance::apply(const RobotState &robotstate) {

    //ROS_INFO("CartesianImpedance::apply");

    Eigen::VectorXd F_task(6);

    if (!is_active_) {
        return;
    }

    //ROS_INFO("    is active");

    if (end_effector_frame_ == "grippoint_left") {
        end_effector_pose_ = robotstate.endEffectorPoseLeft;
        chain_ = robotstate.chain_left_;
    }
    else if (end_effector_frame_ == "grippoint_right") {
        end_effector_pose_ = robotstate.endEffectorPoseRight;
        chain_ = robotstate.chain_right_;
    }

    KDL::Frame end_effector_kdl_frame;
    KDL::Frame goal_kdl_frame;
    Eigen::Vector3d end_effector_RPY;
    Eigen::Vector3d goal_RPY;
    stampedPoseToKDLframe(end_effector_pose_, end_effector_kdl_frame, end_effector_RPY);
    stampedPoseToKDLframe(goal_pose_, goal_kdl_frame, goal_RPY);

    //ROS_INFO("goal_kdl_frame = %f,\t%f,\t%f,\t%f,\t%f,\t%f", goal_kdl_frame.p.x(), goal_kdl_frame.p.y(), goal_kdl_frame.p.z(),
    //                                                         goal_RPY(0), goal_RPY(1), goal_RPY(2));
    //ROS_INFO("end_effector_kdl_frame = %f,\t%f,\t%f,\t%f,\t%f,\t%f", end_effector_kdl_frame.p.x(), end_effector_kdl_frame.p.y(), end_effector_kdl_frame.p.z(),
    //                                                                 end_effector_RPY(0), end_effector_RPY(1), end_effector_RPY(2));

    KDL::Twist error_vector_fk = KDL::diff(end_effector_kdl_frame,goal_kdl_frame, Ts);


    error_vector_(0) = error_vector_fk.vel.x();
    error_vector_(1) = error_vector_fk.vel.y();
    error_vector_(2) = error_vector_fk.vel.z();
    error_vector_(3) = goal_RPY(0) - end_effector_RPY(0)/ Ts;
    error_vector_(4) = goal_RPY(1) - end_effector_RPY(1)/ Ts;
    error_vector_(5) = goal_RPY(2) - end_effector_RPY(2)/ Ts;


    //std::cout << "goal_pose = " << goal_pose_.getOrigin().getX() << " , " << goal_pose_.getOrigin().getY() << " , " << goal_pose_.getOrigin().getZ() << std::endl;
    //std::cout << "error_vector = " << error_vector_ << std::endl;

    // If pre-grasping, an offset is added to the errorpose in x-direction
    //if (pre_grasp_) error_vector_(0) -= pre_grasp_delta_;

    //ROS_INFO("error pose = %f,\t%f,\t%f,\t%f,\t%f,\t%f",error_vector_(0),error_vector_(1),error_vector_(2),error_vector_(3),error_vector_(4),error_vector_(5));

    F_task = K_ * error_vector_;

    //std::cout << "F_task = " << F_task << std::endl;
    //ROS_INFO("F_task = %f,\t%f,\t%f,\t%f,\t%f,\t%f",F_task(0),F_task(1),F_task(2),F_task(3),F_task(4),F_task(5));

    // add the wrench to the end effector of the kinematic chain
    chain_->addCartesianWrench(end_effector_frame_, F_task);


    // ToDo: Include boundaries in action message
    int num_converged_dof = 0;
    for (uint i = 0; i < 3; i++) {
        if (fabs(error_vector_(i)) < 0.035) ++num_converged_dof;
    }
    for (uint i = 0; i < 3; i++) {
        if (fabs(error_vector_(i+3)) < 0.3) ++num_converged_dof;
    }
    if (num_converged_dof == 6 && pre_grasp_) {
        pre_grasp_ = false;
        ROS_INFO("pre_grasp_ = %d",pre_grasp_);
    }
    //////////else if (num_converged_dof == 6 && server_->isActive() && !pre_grasp_) {
    //////////server_->setSucceeded();
    //////////ROS_WARN("errorpose = %f,\t%f,\t%f,\t%f,\t%f,\t%f",error_vector_(0),error_vector_(1),error_vector_(2),error_vector_(3),error_vector_(4),error_vector_(5));
    //////////}
    else if (num_converged_dof == 6 && status_ == 2 && !pre_grasp_) {
        status_ = 1;
        ROS_WARN("errorpose = %f,\t%f,\t%f,\t%f,\t%f,\t%f",error_vector_(0),error_vector_(1),error_vector_(2),error_vector_(3),error_vector_(4),error_vector_(5));
    }


}


void CartesianImpedance::stampedPoseToKDLframe(geometry_msgs::PoseStamped& pose, KDL::Frame& frame, Eigen::Vector3d& RPY) {

    if (pose.header.frame_id != "/base_link") ROS_WARN("FK computation can now only cope with base_link as input frame");

    // Position
    frame.p.x(pose.pose.position.x);
    frame.p.y(pose.pose.position.y);
    frame.p.z(pose.pose.position.z);

    // Orientation
    frame.M.Quaternion(pose.pose.orientation.x,
                       pose.pose.orientation.y,
                       pose.pose.orientation.z,
                       pose.pose.orientation.w);

    tf::Quaternion q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    RPY(0) = roll;
    RPY(1) = pitch;
    RPY(2) = yaw;

}
