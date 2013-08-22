#include "CartesianImpedance.h"
#include <tf/transform_datatypes.h>

#include <amigo_whole_body_controller/ArmTaskAction.h>
#include <actionlib/server/simple_action_server.h>

CartesianImpedance::CartesianImpedance(const std::string& tip_frame) :
    chain_(0) {

    type_ = "CartesianImpedance";
    tip_frame_ = tip_frame;

    ROS_INFO("Initializing Cartesian Impedance for %s", tip_frame_.c_str());

    /// Set stiffness matrix to zero
    K_.resize(6,6);
    K_.setZero();
    num_constrained_dofs_ = 0;
    for (unsigned int i = 0; i < 3; i++) {
        box_tolerance_[i] = 0;
        orientation_tolerance_[i] = 0;
    }

    pose_error_.Zero();

    ROS_INFO("Initialized Cartesian Impedance");
}

bool CartesianImpedance::initialize(RobotState &robotstate) {
    return true;
}

CartesianImpedance::~CartesianImpedance() {
}

void CartesianImpedance::setGoal(const geometry_msgs::PoseStamped& goal_pose ) {

    /// Root frame
    root_frame_ = goal_pose.header.frame_id;

    /// Goal Pose
    //////goal_pose_ = goal_pose;
    stampedPoseToKDLframe(goal_pose, goal_pose_);
}

void CartesianImpedance::setImpedance(const geometry_msgs::Wrench &stiffness) {

    /// Copy desired stiffness into K matrix
    K_(0,0) = stiffness.force.x;
    K_(1,1) = stiffness.force.y;
    K_(2,2) = stiffness.force.z;
    K_(3,3) = stiffness.torque.x;
    K_(4,4) = stiffness.torque.y;
    K_(5,5) = stiffness.torque.z;

    /// Count the number of constrained DoFs
    num_constrained_dofs_ = 0;
    for (unsigned int i = 0; i < 6; i++) {
        if (K_(i,i) != 0) {
            num_constrained_dofs_ += 1;
        }
    }
}

bool CartesianImpedance::setPositionTolerance(const arm_navigation_msgs::Shape &position_tolerance) {

    if (position_tolerance.type != 1) {
        ROS_WARN("Other position tolerance than box not yet implemented, defaulting");
        return false;
    }
    else {
        box_tolerance_[0] = position_tolerance.dimensions[0];
        box_tolerance_[1] = position_tolerance.dimensions[1];
        box_tolerance_[2] = position_tolerance.dimensions[2];
        return true;
    }
}

bool CartesianImpedance::setOrientationTolerance(const float roll, const float pitch, const float yaw) {
    orientation_tolerance_[0] = roll;
    orientation_tolerance_[1] = pitch;
    orientation_tolerance_[2] = yaw;
    return true;
}

void CartesianImpedance::cancelGoal() {
    status_ = 0;
}

void CartesianImpedance::apply(RobotState &robotstate) {

    //ROS_INFO("CartesianImpedance::apply");

    Eigen::VectorXd F_task(6);
    Eigen::VectorXd error_vector(6);

    // ToDo: create get function
    /// Get end effector pose (is in map frame)
    std::map<std::string, KDL::Frame>::iterator itrFK = robotstate.fk_poses_.find(tip_frame_);
    end_effector_pose_ = (*itrFK).second;
    //ROS_INFO("FK pose tip frame in tree root = (%f,%f,%f)", end_effector_pose_.p.x(), end_effector_pose_.p.y(), end_effector_pose_.p.z());

    /// Get the pose of the root frame (of the goal) in map
    std::map<std::string, KDL::Frame>::iterator itrRF = robotstate.fk_poses_.find(root_frame_);
    KDL::Frame kdl_root_in_map = itrRF->second;
    //ROS_INFO("FK pose root frame in map (x,y,z) = (%f,%f,%f)", kdl_root_in_map.p.x(), kdl_root_in_map.p.y(), kdl_root_in_map.p.z());

    /// Convert the goal pose to map (is required because pose of robot in map (amcl_pose) may have been updated)
    KDL::Frame goal_pose_map = goal_pose_* kdl_root_in_map;
    //ROS_INFO("goal_pose_map (x,y,z) = (%f,%f,%f)",goal_pose_map.p.x(),goal_pose_map.p.y(),goal_pose_map.p.z());

    /// Compute pose error
    pose_error_ = KDL::diff(end_effector_pose_, goal_pose_map);

    error_vector(0) = pose_error_.vel.x();
    error_vector(1) = pose_error_.vel.y();
    error_vector(2) = pose_error_.vel.z();
    error_vector(3) = pose_error_.rot.x();//goal_RPY(0) - end_effector_RPY(0)/ Ts;
    error_vector(4) = pose_error_.rot.y();//goal_RPY(1) - end_effector_RPY(1)/ Ts;
    error_vector(5) = pose_error_.rot.z();//goal_RPY(2) - end_effector_RPY(2)/ Ts;

    //std::cout << "goal_pose = " << goal_pose_.getOrigin().getX() << " , " << goal_pose_.getOrigin().getY() << " , " << goal_pose_.getOrigin().getZ() << std::endl;
    //std::cout << "error_vector = " << error_vector << std::endl;

    F_task = K_ * error_vector;

    //std::cout << "K_ = " << K_ << std::endl;
    //std::cout << "F_task = " << F_task << std::endl;
    //ROS_INFO("Tip frame = %s",tip_frame_.c_str());

    // add the wrench to the end effector of the kinematic chain
    robotstate.tree_.addCartesianWrench(tip_frame_,F_task);

    /// Count number of converged DoFs
    // ToDo: Include boundaries in action message
    // ToDo: Include other checks than box check (move this to separate function)
    unsigned int num_converged_dof = 0;
    for (unsigned int i = 0; i < 3; i++) {
        /// Only take constrained DoFs into account
        /// Position
        if (K_(i,i) != 0) {
            if (fabs(error_vector(i)) < box_tolerance_[i]/2) ++num_converged_dof;
        }
        /// Orientation
        if (K_(i+3,i+3) != 0) {
            if (fabs(error_vector(i+3)) < orientation_tolerance_[i]) ++num_converged_dof;
        }
    }

    if (num_converged_dof == num_constrained_dofs_ && status_ == 2) {
        status_ = 1;
        ROS_WARN("errorpose = %f,\t%f,\t%f,\t%f,\t%f,\t%f",error_vector(0),error_vector(1),error_vector(2),error_vector(3),error_vector(4),error_vector(5));
    }
}

unsigned int CartesianImpedance::getStatus() {
    return status_;
}

KDL::Twist CartesianImpedance::getError() {
    return pose_error_;
}

void CartesianImpedance::stampedPoseToKDLframe(const geometry_msgs::PoseStamped& pose, KDL::Frame& frame) {

    //if (pose.header.frame_id != "base_link") //ROS_WARN("FK computation can now only cope with base_link as input frame, while it currently is %s", pose.header.frame_id.c_str());

    // Position
    frame.p.x(pose.pose.position.x);
    frame.p.y(pose.pose.position.y);
    frame.p.z(pose.pose.position.z);

    // Orientation
    frame.M.Quaternion(pose.pose.orientation.x,
                       pose.pose.orientation.y,
                       pose.pose.orientation.z,
                       pose.pose.orientation.w);

}
