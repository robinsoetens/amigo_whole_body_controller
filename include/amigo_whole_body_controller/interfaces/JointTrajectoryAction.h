/*!
 * \author Janno Lunenburg
 * \date October, 2012
 * \version 0.1
 */

#ifndef JOINTTRAJECTORYACTION_H
#define JOINTTRAJECTORYACTION_H

#include <ros/ros.h>
//#include <actionlib/server/action_server.h>
#include <actionlib/server/simple_action_server.h>

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

    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> action_server;
    typedef action_server::GoalHandle GoalHandle;

    action_server *server_, *server_left_, *server_right_;

    control_msgs::FollowJointTrajectoryGoal active_goal_;

    /** Indicates from which server the active_goal comes from */
    std::string recent_server_;

    /** Pointer to whole body controller object */
    WholeBodyController *wbc_;

    /** Index of joint trajectory */
    unsigned int trajectory_index_;

    /** Time of receiving goal */
    ros::Time goal_reception_time_;

    /** Vector containing joint names */
    std::vector<std::string> joint_names_;

    /** Map fron joint names to joint index */
    std::map<std::string, unsigned int> joint_index_;

    /** Intermediate goal constraints */
    std::map<std::string,double> intermediate_goal_constraints_;

    /** Final goal constraints */
    std::map<std::string,double> final_goal_constraints_;

    /** Trajectory constraints */
    std::map<std::string,double> trajectory_constraints_;

    /** Goal time constraint */
    double goal_time_constraint_;

    /** Callback function for joint goal */
    void goalCB();
    void goalCBLeft();
    void goalCBRight();

    /** Callback function for cancel goal */
    void cancelCB();

    /** Sets one of the action servers succeeded */
    void setSucceeded();

    /** Sets one of the action servers aborted */
    void setAborted();

    /** Set joint positions */
    bool setJointPositions();

};

#endif // JOINTTRAJECTORYACTION_H
