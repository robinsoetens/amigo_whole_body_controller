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
#include <geometry_msgs/Wrench.h>
#include <arm_navigation_msgs/Shape.h>

// Eigen
#include <Eigen/Core>

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
    CartesianImpedance(const std::string& tip_frame);

    /**
     * Deconstructor
     */
    virtual ~CartesianImpedance();

    bool initialize(RobotState &robotstate);

    void apply(RobotState& robotstate);

    void setGoal(const geometry_msgs::PoseStamped &goal_pose);

    void setImpedance(const geometry_msgs::Wrench &stiffness);

    bool setPositionTolerance(const arm_navigation_msgs::Shape &position_tolerance);

    bool setOrientationTolerance(const float roll, const float pitch, const float yaw);

    void cancelGoal();

    unsigned int getStatus();

    KDL::Twist getError();


protected:

    //! Sampling time
    double Ts;

    // ToDo: SHOULD be obsolete
    Chain* chain_;

    //////geometry_msgs::PoseStamped end_effector_pose_,goal_pose_;
    KDL::Frame end_effector_pose_,goal_pose_;

    KDL::Twist pose_error_;

    // Converts Converts geometry_msgs::PoseStamped to KDL::Frame
    void stampedPoseToKDLframe(const geometry_msgs::PoseStamped& pose, KDL::Frame& frame);

    // Cartesian Impedance matrix
    Eigen::MatrixXd K_;

    /**
      * Indicates the number of constrained degrees of freedom
      * If a zero stiffness in a certain direction is selected, this implies that this DoF need not be constrained
      */
    unsigned int num_constrained_dofs_;

    /**
      * Array containing the dimensions of the box for the position constraint
      * A DoF is converged if error < (dimension/2)
      */
    double box_tolerance_[3];

    /**
      * Array containing the absolute_roll_tolerance, absolute_pitch_tolerance and absolute_yaw_tolerance
      * A DoFs is converged if error < tolerance
      */
    double orientation_tolerance_[3];

    // Status for the Cartesian Impedance
    // IDLE=0
    // AT_GOAL_POSE=1
    // MOVING_TO_GOAL_POSE=2
    unsigned int status_;


};

#endif
