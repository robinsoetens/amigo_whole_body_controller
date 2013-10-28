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
#include "WholeBodyController.h"

class JointTrajectoryAction {

public:

    /**
      * Constructor
      */
    JointTrajectoryAction(WholeBodyController *wbc);

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

    //action_server* server_;//(node_, "/grasp_precompute_left", boost::bind(&execute, _1, &server, &client), false);
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *server_;
    GoalHandle active_goal_;

    /** Pointer to whole body controller object */
    WholeBodyController *wbc_;

    /** Index of joint trajectory */
    unsigned int trajectory_index_;

    /** Time of receiving goal */
    ros::Time goal_reception_time_;

    /** Intermediate goal constraints */
    std::map<std::string,double> intermediate_goal_constraints_;

    /** Final goal constraints */
    std::map<std::string,double> final_goal_constraints_;

    /** Trajectory constraints */
    std::map<std::string,double> trajectory_constraints_;

    /** Goal time constraint */
    double goal_time_constraint_;

    /**
      * Callback function for Cartesian goal
      */
    void goalCB(GoalHandle gh);

    /**
      * Callback function for cancel goal
      */
    void cancelCB(GoalHandle gh);

    /** Set joint positions */
    bool setJointPositions();

};

#endif // JOINTTRAJECTORYACTION_H
