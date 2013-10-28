#include <amigo_whole_body_controller/interfaces/JointTrajectoryAction.h>

JointTrajectoryAction::JointTrajectoryAction(WholeBodyController *wbc) {
    wbc_ = wbc;
    initialize();
}

JointTrajectoryAction::~JointTrajectoryAction() {
    delete server_;
}

bool JointTrajectoryAction::initialize() {

    trajectory_index_ = 0;

    ros::NodeHandle nh;
    /*
    server_ = new action_server(nh, "joint_trajectory_action", false);
    //server_->registerGoalCallback(boost::bind(&JointTrajectoryAction::goalCB(), this));
    server_->registerGoalCallback(boost::bind(&JointTrajectoryAction::goalCB, this));
    //server_->registerCancelCallback(boost::bind(&JointTrajectoryAction::cancelCB(), this));
    server_->start();
    //action_server_ = new actionlib::SimpleActionServer<amigo_whole_body_controller::ArmTaskAction>(nh_private, "motion_constraint", false);
    //action_server_->registerGoalCallback(boost::bind(&WholeBodyPlanner::goalCB, this));
    //action_server_->start();
    */
    server_ = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(nh, "joint_trajectory_action", false);
    //server_->registerGoalCallback(boost::bind(&JointTrajectoryAction::goalCB, this));
    server_->start();

    // ToDo: fill constraints etc.

    return true;
}

void JointTrajectoryAction::update() {

    /// Only do stuff when required
    if (is_active_)
    {
        /// Check time constraint
        // We never do this?

        /// Loop over joints
        /// Check trajectory, intermediate, final goal constraints, convergence
        unsigned int converged_joints = 0;
        for (unsigned int i = 0; i < active_goal_.getGoal()->trajectory.joint_names.size(); i++)
        {
            std::string joint_name = active_goal_.getGoal()->trajectory.joint_names[i];
            double ref = active_goal_.getGoal()->trajectory.points[trajectory_index_].positions[i];
            double pos = wbc_->getJointPosition(joint_name);
            double abs_error = ref-pos;

            /// Check trajectory constraints
            if(abs_error > trajectory_constraints_[joint_name]) {
                ROS_WARN("Aborting because the trajectory constraint was violated");
                //for (unsigned int j = 0; j < number_of_goal_joints_; j++) {
                //    if ( fabs(ref_pos_[j] - cur_pos_[j]) > intermediate_goal_constraints_[joint_names_[j]]) {
                //        ROS_WARN("Error joint %s = %f exceeds intermediate joint constraint (%f)",joint_names_[j].c_str(),ref_pos_[j] - cur_pos_[j],intermediate_goal_constraints_[joint_names_[j]]);
                //    }
                //    else if ( fabs(ref_pos_[j] - cur_pos_[j]) > final_goal_constraints_[joint_names_[j]]) {
                //        ROS_WARN("Error joint %s = %f exceeds final joint contraint (%f)",joint_names_[j].c_str(),ref_pos_[j] - cur_pos_[j],final_goal_constraints_[joint_names_[j]]);
                //    }
                }
                active_goal_.setAborted();
                is_active_=false;
                return;

            /// Check if this joint has converged
            if(trajectory_index_ < ((int)active_goal_.getGoal()->trajectory.points.size()-1))
            {
                if(abs_error < intermediate_goal_constraints_[joint_name])
                {
                    converged_joints += 1;
                }
            }
            else
            {
                if(abs_error < final_goal_constraints_[joint_name])
                {
                    converged_joints += 1;
                }
            }

            /// Set joint position in whole-body controller
            wbc_->setDesiredJointPosition(joint_name, ref);
        }

        /// Check whether all joints of this point have converged
        if (converged_joints == active_goal_.getGoal()->trajectory.joint_names.size())
        {
            trajectory_index_ += 1;
        }

        /// Check whether the final goal is achieved
        if (trajectory_index_ == active_goal_.getGoal()->trajectory.points.size())
        {
            active_goal_.setSucceeded();
            is_active_ = false;
        }
    }
}

void JointTrajectoryAction::goalCB(GoalHandle gh) {

    ROS_INFO("Received new joint goal");
    trajectory_index_ = 0;
    goal_reception_time_ = ros::Time::now();

    // Cancels the currently active goal (if there is one).
    if (active_goal_.getGoalStatus().status == 1) {
        // Stop something???

        // Marks the current goal as canceled.
        active_goal_.setCanceled();
        is_active_ = false;
        ROS_INFO("Canceling previous goal");
    }

    // ToDo: Check whether the received goal meets it requirements
    /*if () {
            ROS_ERROR("Joint goal does not meet requirements");
            gh.setRejected();
        }*/
    // else {
    // Set goal to accepted etc;
    gh.setAccepted();
    active_goal_ = gh;
    is_active_ = true;

    // ToDo: resize certain vectors
    //}

}

void JointTrajectoryAction::cancelCB(GoalHandle gh) {

    if (active_goal_ == gh)
    {
        /// Marks the current goal as canceled.
        active_goal_.setCanceled();
        is_active_ = false;
        ROS_INFO("Canceling current goal");
    }


}

bool JointTrajectoryAction::setJointPositions()
{
    /// Loop over joints
    for (unsigned i = 0; i < active_goal_.getGoal()->trajectory.joint_names.size(); i++)
    {
        wbc_->setDesiredJointPosition(active_goal_.getGoal()->trajectory.joint_names[i], active_goal_.getGoal()->trajectory.points[trajectory_index_].positions[i]);
    }
    return true;
}
