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
    CartesianImpedance(const std::string& end_effector_frame, tf::TransformListener* tf_listener);

    /**
     * Deconstructor
     */
    virtual ~CartesianImpedance();

    bool initialize();

    bool isActive();

    void apply(const RobotState& robotstate);

    void setGoal(tf::Stamped<tf::Pose>& goal_pose);

    void cancelGoal();

    // Status for the Cartesian Impedance
    // IDLE=0
    // AT_GOAL_POSE=1
    // MOVING_TO_GOAL_POSE=2
    unsigned int status_;


protected:

    //! Bool indicating whether this is active
    bool is_active_;

    Chain* chain_;

    std::string end_effector_frame_;

    tf::Stamped<tf::Pose> goal_pose_;

    // Cartesian Impedance matrix
    Eigen::MatrixXd K_;

    tf::Stamped<tf::Pose> error_pose_;

    Eigen::VectorXd error_vector_;

    bool pre_grasp_;
    double pre_grasp_delta_;

    tf::TransformListener* tf_listener_;


};

#endif
