#include "JointTrajectoryAction.h"

JointTrajectoryAction::JointTrajectoryAction() {

}

JointTrajectoryAction::~JointTrajectoryAction() {

}

JointTrajectoryAction::initialize() {

}

JointTrajectoryAction::update() {

}

JointTrajectoryAction::goalCB() {

    ROS_INFO("Received new joint goal");

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

JointTrajectoryAction::cancelCB() {

    if (active_goal_ == gh)
    {
        // Marks the current goal as canceled.
        active_goal_.setCanceled();
        is_active_ = false;
        ROS_INFO("Canceling current goal");
    }


}
