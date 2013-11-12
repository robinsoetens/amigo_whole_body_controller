/*!
 * \author Janno Lunenburg
 * \date September, 2012
 * \version 0.1
 */

#ifndef CARTESIANIMPEDANCE_H_
#define CARTESIANIMPEDANCE_H_

#include "MotionObjective.h"
//#include "CollisionAvoidance.h"

// ROS
//#include "ros/ros.h"

// Messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>
#include <arm_navigation_msgs/Shape.h>

// Eigen
#include <Eigen/Core>

// tf
//#include <tf/transform_listener.h>

// Action server
//#include <actionlib/server/action_server.h>
//#include <actionlib/server/simple_action_server.h>

// Action definition
//#include <amigo_arm_navigation/grasp_precomputeAction.h>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

//
#include <fstream>
#include <amigo_ref_interpolator/interpolator.h>

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

    void setGoal(const geometry_msgs::PoseStamped &goal_pose );

    void setGoalOffset(const geometry_msgs::Point &target_point_offset);

    void setImpedance(const geometry_msgs::Wrench &stiffness);

    bool setPositionTolerance(const arm_navigation_msgs::Shape &position_tolerance);

    bool setOrientationTolerance(const float roll, const float pitch, const float yaw);

    void cancelGoal();

    unsigned int getStatus();

    KDL::Twist getError();

    std::vector<refgen::RefGenerator> ref_generators;

protected:

    //! Sampling time
    double Ts;

    // ToDo: SHOULD be obsolete
    Chain* chain_;

    ros::Publisher pub_CI_wrench_;

    //////geometry_msgs::PoseStamped end_effector_pose_,goal_pose_;
    KDL::Frame Frame_root_goal_;

    KDL::Twist pose_error_;

    // Maximum magnitude of the attractive force;
    double f_max_;

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
      * Position constraint type
      * 0 = Sphere, 1 = Box, 2 = Cylinder and 3 = Mesh
      */
    unsigned int constraint_type_;

    /**
      * Array containing the dimensions of the box for the position constraint
      * A DoF is converged if error < (dimension/2)
      */
    double box_tolerance_[3];
    /**
      * Array containing the radius of the sphere for the position constraint
      * A DoF is converged if error is inside sphere, error_x^2 + error_y^2 + error_z^2 < radius
      */
    double sphere_tolerance_;

    /**
      * Array containing the radius and length of the cylinder for the position constraint
      * A DoF is converged if error is inside cylinder, error_x^2 + error_y^2 + error_z^2 < radius
      */
    double cylinder_tolerance_[2];


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

    /**
      *
      *
      */
    unsigned int convergedConstraints();

    /**
      *
      *
      */
    void refGeneration(KDL::Frame& goal, KDL::Frame& ref);

    /**
      * Frame for specifiying offset from tip, for pre-grasp
      *
      */
    KDL::Frame Frame_tip_offset;

    /**
      * Reference point for offset of tip, for pre-grasp
      *
      */
    KDL::Vector ref_tip_offset;

};

#endif
