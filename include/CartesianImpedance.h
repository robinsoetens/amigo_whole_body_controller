/*!
 * \author Janno Lunenburg
 * \date September, 2012
 * \version 0.1
 */

#ifndef CARTESIANIMPEDANCE_H_
#define CARTESIANIMPEDANCE_H_

#include "MotionObjective.h"

// ROS
#include "ros/ros.h"

// Messages
#include <geometry_msgs/PoseStamped.h>

// tf
#include <tf/transform_listener.h>

// Action server
#include <actionlib/server/action_server.h>
#include <actionlib/server/simple_action_server.h>

// Action definition
#include <amigo_arm_navigation/grasp_precomputeAction.h>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

/////

class CartesianImpedance : public MotionObjective {

public:

    /**
     * Constructor
     */
    CartesianImpedance(const std::string& end_effector_frame);

    /**
     * Deconstructor
     */
    virtual ~CartesianImpedance();

    bool initialize();

    bool isActive();

    void apply(const RobotState& robotstate);

    void setGoal(geometry_msgs::PoseStamped &goal_pose);

    void cancelGoal();

    // Status for the Cartesian Impedance
    // IDLE=0
    // AT_GOAL_POSE=1
    // MOVING_TO_GOAL_POSE=2
    unsigned int status_;


protected:

    //! Sampling time
    double Ts;

    //! Bool indicating whether this is active
    bool is_active_;

    Chain* chain_;

    std::string end_effector_frame_;

    geometry_msgs::PoseStamped end_effector_pose_;
    geometry_msgs::PoseStamped goal_pose_;
    geometry_msgs::PoseStamped error_pose_;

    // Converts Converts geometry_msgs::PoseStamped to KDL::Frame
    void stampedPoseToKDLframe(geometry_msgs::PoseStamped& pose, KDL::Frame& frame);

    // Cartesian Impedance matrix
    Eigen::MatrixXd K_;

    Eigen::VectorXd error_vector_;

    bool pre_grasp_;
    double pre_grasp_delta_;


};

#endif
