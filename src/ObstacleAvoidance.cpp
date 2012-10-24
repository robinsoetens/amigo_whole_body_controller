#include "ObstacleAvoidance.h"
#include <tf/transform_datatypes.h>

ObstacleAvoidance::ObstacleAvoidance() {

}

ObstacleAvoidance::~ObstacleAvoidance() {
}

bool ObstacleAvoidance::initialize(const std::string& end_effector_frame, uint F_start_index) {

    ROS_INFO("Initializing Obstacle Avoidance");

    // Get node handle
    ros::NodeHandle n("~");
    std::string ns = ros::this_node::getName();    

    end_effector_frame_ = end_effector_frame;
    F_start_index_ = F_start_index;

    bool wait_for_transform = listener_.waitForTransform(end_effector_frame_,"/map", ros::Time::now(), ros::Duration(1.0));
    if (!wait_for_transform) ROS_WARN("Transform between %s and /map is not available",end_effector_frame.c_str());

    ROS_INFO("Initialized Obstacle Avoidance");

    return true;
}

void ObstacleAvoidance::update(Eigen::VectorXd& F_task, uint& force_vector_index) {

    tf::Stamped<tf::Pose> end_effector_pose;
    end_effector_pose.setIdentity();
    end_effector_pose.frame_id_ = end_effector_frame_;
    end_effector_pose.stamp_ = ros::Time();

    tf::Stamped<tf::Pose> end_effector_pose_MAP;

    try {
        listener.transformPose("/map", end_effector_pose, end_effector_pose_MAP);
    } catch (tf::TransformException& e) {
        ROS_ERROR("CartesianImpedance: %s", e.what());
        return;
    }



    //listener.transformPose(end_effector_frame_,now,goal_pose_,goal_pose_.header.frame_id,errorPose);

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

    // If pre-grasping, an offset is added to the errorpose in x-direction
    if (pre_grasp_) error_vector_(0) -= pre_grasp_delta_;

    ///ROS_INFO("errorpose = %f,\t%f,\t%f,\t%f,\t%f,\t%f",error_vector_(0),error_vector_(1),error_vector_(2),error_vector_(3),error_vector_(4),error_vector_(5));

    F_task.segment(force_vector_index,6) = K_ * error_vector_;
    force_vector_index += 6;

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
    else if (num_converged_dof == 6 && server_->isActive() && !pre_grasp_) {
        server_->setSucceeded();
        ROS_WARN("errorpose = %f,\t%f,\t%f,\t%f,\t%f,\t%f",error_vector_(0),error_vector_(1),error_vector_(2),error_vector_(3),error_vector_(4),error_vector_(5));
    }


}

/////
void CartesianImpedance::goalCB() {

    const amigo_arm_navigation::grasp_precomputeGoal& goal = *server_->acceptNewGoal();

    ROS_WARN("Received new goal");

    /*
    // Cancels the currently active goal.
    if (active_goal_.getGoalStatus().status == 1) {
        // Stop something???

        // Marks the current goal as canceled.
        active_goal_.setCanceled();
        is_active_ = false;
        ROS_WARN("Canceling previous goal");
    }
    */

    // ToDo: Check whether the received goal meets it requirements
    /*if (gh.getGoal()->PERFORM_PRE_GRASP) {
        ROS_ERROR("Pre-grasp not yet implemented, rejecting goal");
        gh.setRejected();
    }*/
    //else {

    // Put the goal data in the right datatype
    goal_pose_.header = goal.goal.header;
    goal_pose_.pose.position.x = goal.goal.x;
    goal_pose_.pose.position.y = goal.goal.y;
    goal_pose_.pose.position.z = goal.goal.z;
    double roll = goal.goal.roll;
    double pitch = goal.goal.pitch;
    double yaw = goal.goal.yaw;
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    goal_pose_.pose.orientation = orientation;

    // Pre-grasp
    if (goal.PERFORM_PRE_GRASP) {
        pre_grasp_ = true;
        ROS_INFO("pre_grasp_ = %d",pre_grasp_);
    }

    // Set goal to accepted etc;
    //gh.setAccepted();
    //active_goal_ = gh;
    is_active_ = true;
    //}



}

void CartesianImpedance::cancelCB() {

    /*
    if (active_goal_ == gh)
    {
        // Marks the current goal as canceled.
        active_goal_.setCanceled();
        is_active_ = false;
    }
    */

    is_active_ = false;
    server_->setPreempted();
}

/////
