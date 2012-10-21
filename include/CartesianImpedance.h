/*!
 * \author Janno Lunenburg
 * \date September, 2012
 * \version 0.1
 */

#ifndef CARTESIANIMPEDANCE_H_
#define CARTESIANIMPEDANCE_H_

// ROS
#include "ros/ros.h"

// Messages
#include <geometry_msgs/PoseStamped.h>

// tf
#include <tf/transform_listener.h>

// Eigen
#include <Eigen/Core>

/////
// Action server
#include <actionlib/server/action_server.h>
#include <actionlib/server/simple_action_server.h>

// Action definition
#include <amigo_arm_navigation/grasp_precomputeAction.h>


/////

class CartesianImpedance {

public:

    //ToDo: make configure, start- and stophook. Components can be started/stopped in an actionlib kind of fashion

    /**
     * Constructor
     */
    CartesianImpedance();

    /**
     * Deconstructor
     */
    virtual ~CartesianImpedance();

    /**
     * Initialize function
     */
    bool initialize(const std::string&, uint);

    /**
     * Update
     */
    void update(Eigen::VectorXd&, uint& force_vector_index);

    //! Bool indicating whether this is active
    bool is_active_;

    /////

    /////

protected:

    /**
     * Subscriber to target position
     */
    ros::Subscriber target_sub_;

    void callbackTarget(const geometry_msgs::PoseStamped::ConstPtr& msg);

    tf::TransformListener listener;

    /////geometry_msgs::PoseStamped transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped poseMsg);
    geometry_msgs::PoseStamped transformPose(geometry_msgs::PoseStamped poseMsg);

    geometry_msgs::PoseStamped errorPose;
    Eigen::VectorXd error_vector_;

    // Cartesian Impedance matrix
    Eigen::MatrixXd K_;

    std::string end_effector_frame_;

    uint F_start_index_;

    /////
    typedef actionlib::ActionServer<amigo_arm_navigation::grasp_precomputeAction> action_server;
    typedef action_server::GoalHandle GoalHandle;

    ///ros::NodeHandle nh_;//("~");
    //action_server action_server_;
    action_server* server_;//(node_, "/grasp_precompute_left", boost::bind(&execute, _1, &server, &client), false);
    GoalHandle active_goal_;

    geometry_msgs::PoseStamped goal_pose_;

    /**
      * Callback function for Cartesian goal
      */
    void goalCB(GoalHandle gh);

    /**
      * Callback function for cancel goal
      */
    void cancelCB(GoalHandle gh);

    bool pre_grasp_;
    double pre_grasp_delta_;

    //void execute(const amigo_arm_navigation::grasp_precomputeGoalConstPtr& goal_in, action_server* as);
    /////

};

#endif
