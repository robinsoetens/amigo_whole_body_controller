/*!
 * \author Janno Lunenburg
 * \date September, 2012
 * \version 0.1
 */

#ifndef CARTESIANIMPEDANCE_H_
#define CARTESIANIMPEDANCE_H_

#include "MotionObjective.h"

// Messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>
#include <arm_navigation_msgs/Shape.h>

// Eigen
#include <Eigen/Core>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

//
#include <fstream>
#include <amigo_ref_interpolator/interpolator.h>
#include <tue_gazebo_plugins/ReferenceGenerator.h>
#include "amigo_whole_body_controller/Tracing.hpp"

class CartesianImpedance : public MotionObjective {

public:

    /** Constructor */
    CartesianImpedance(const std::string& tip_frame, const double Ts);

    /** Deconstructor */
    virtual ~CartesianImpedance();

    bool initialize(RobotState &robotstate);

    void apply(RobotState& robotstate);

    void setGoal(const geometry_msgs::PoseStamped &goal_pose );

    void setGoalOffset(const geometry_msgs::Point &target_point_offset);

    void setImpedance(const geometry_msgs::Wrench &stiffness);

    bool setPositionTolerance(const arm_navigation_msgs::Shape &position_tolerance);

    bool setOrientationTolerance(const float roll, const float pitch, const float yaw);

    void setVelocity(RobotState &robotstate);

    void cancelGoal();

    KDL::Twist getError();

    std::vector<refgen::RefGenerator> ref_generators;

    std::vector<controller::RefGenerator> ref_generator_;

protected:

    //! Sampling time
    double Ts_;

    /** Pose of the goal in root frame */
    KDL::Frame frame_root_goal_;

    /** Pose error */
    KDL::Twist pose_error_;

    /** End-effectory velocity in map frame */
    KDL::Twist ee_map_vel_;

    /** Matrix containing the combined Jacobian matrix of this objective */
    Eigen::MatrixXd jacobian_pre_alloc_;
    /** Vector containing all relevant wrench entries */
    Eigen::VectorXd wrenches_pre_alloc_;

    // Converts Converts geometry_msgs::PoseStamped to KDL::Frame
    void stampedPoseToKDLframe(const geometry_msgs::PoseStamped& pose, KDL::Frame& frame);

    // Cartesian Impedance and Damping matrices
    Eigen::MatrixXd K_, D_;

    /** Indicates the number of constrained degrees of freedom
      * If a zero stiffness in a certain direction is selected, this implies that this DoF need not be constrained */
    unsigned int num_constrained_dofs_;

    /** Position constraint type
      * 0 = Sphere, 1 = Box, 2 = Cylinder and 3 = Mesh */
    unsigned int constraint_type_;

    /** Array containing the dimensions of the box for the position constraint
      * A DoF is converged if error < (dimension/2) */
    double box_tolerance_[3];

    /** Array containing the radius of the sphere for the position constraint
      * A DoF is converged if error is inside sphere, error_x^2 + error_y^2 + error_z^2 < radius */
    double sphere_tolerance_;

    /** Array containing the radius and length of the cylinder for the position constraint
      * A DoF is converged if error is inside cylinder, error_x^2 + error_y^2 + error_z^2 < radius*/
    double cylinder_tolerance_[2];

    /** Array containing the absolute_roll_tolerance, absolute_pitch_tolerance and absolute_yaw_tolerance
      * A DoFs is converged if error < tolerance */
    double orientation_tolerance_[3];

    /**      */
    unsigned int convergedConstraints();

    /**      */
    void refGeneration(KDL::Frame& goal, KDL::Frame& ref);

    /** Frame for specifiying offset from tip, for pre-grasp */
    KDL::Frame frame_tip_offset;

    /** Frame for previous tip frame and last measured time */
    KDL::Frame frame_root_tip_previous_, frame_map_ee_previous_;

    /** Reference point for offset of tip, for pre-grasp */
    //KDL::Vector ref_tip_offset;

    /** Tracing object */
    Tracing tracer_;

};

#endif
