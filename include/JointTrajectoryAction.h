/*!
 * \author Janno Lunenburg
 * \date October, 2012
 * \version 0.1
 */

#ifndef JOINTTRAJECTORYACTION_H
#define JOINTTRAJECTORYACTION_H

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <amigo_msgs/arm_joints.h>
#include <amigo_msgs/spindle_setpoint.h>
#include <std_msgs/Float64.h>

class JointTrajectoryAction {

public:

    /**
      * Constructor
      */
    JointTrajectoryAction();

    /**
      * Deconstructor
      */
    virtual ~JointTrajectoryAction();

    /**
     * Initialize function
     */
    bool initialize();

    /**
     * Update
     */
    void update();

    //! Bool indicating whether this is active
    bool is_active_;

private:

    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> action_server;
    typedef action_server::GoalHandle GoalHandle;

    action_server* server_;//(node_, "/grasp_precompute_left", boost::bind(&execute, _1, &server, &client), false);
    GoalHandle active_goal_;

    /**
      * Callback function for Cartesian goal
      */
    void goalCB(GoalHandle gh);

    /**
      * Callback function for cancel goal
      */
    void cancelCB(GoalHandle gh);

};

#endif // JOINTTRAJECTORYACTION_H
