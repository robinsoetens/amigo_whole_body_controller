/*!
 * \author Janno Lunenburg
 * \date September, 2012
 * \version 0.1
 */

#ifndef WHOLEBODYCONTROLLER_H_
#define WHOLEBODYCONTROLLER_H_

// ROS
#include "ros/ros.h"

// Messages
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
///#include <amigo_msgs/arm_joints.h>
///#include <amigo_msgs/spindle_setpoint.h>
///#include <amigo_msgs/head_ref.h>

// WholeBodyController
#include "CartesianImpedance.h"
#include "ComputeJacobian.h"
#include "AdmittanceController.h"
#include "ComputeNullspace.h"
#include "JointLimitAvoidance.h"
#include "PostureControl.h"
#include "ObstacleAvoidance.h"
//#include "TreeDescription.h"

// Vector
#include <vector>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

class WholeBodyController {

public:

    /**
     * Constructor
     */
    WholeBodyController();

    /**
     * Deconstructor
     */
    virtual ~WholeBodyController();

    /*
     * Updatehook
     */
    bool update();

    void setMeasuredJointPosition(const std::string& joint_name, double pos);

    const Eigen::VectorXd& getJointReferences() const;

    const std::vector<std::string>& getJointNames() const;

    bool addMotionObjective(MotionObjective* motionobjective);

    /**
      * Returns the current FK solution
      *
      */
    void getFKsolution(KDL::Frame &FK_end_effector_pose, geometry_msgs::PoseStamped& pose);

protected:

    RobotState robot_state_;

    std::vector<Chain*> chains_;

    std::map<std::string, unsigned int> joint_name_to_index_;

    std::vector<std::string> index_to_joint_name_;

    // The total measured joint array
    KDL::JntArray q_current_;

    KDL::JntArray q_min_;

    KDL::JntArray q_max_;

    //! Unsigned integer containing the total number of joints
    uint num_joints_;

    std::vector<MotionObjective*> motionobjectives_;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    AdmittanceController AdmitCont_;
    ComputeNullspace ComputeNullspace_;
    JointLimitAvoidance JointLimitAvoidance_;
    PostureControl PostureControl_;

    //! Map contains a string to describe the (root joint of the component) this concerns and a vector (is sufficient) with the current joint values
    //std::map<std::string, std::vector<double> > q_current_map_;
    //std::map<std::string, Component*> component_map_;
    //std::map<std::string, uint> joint_name_index_map_;

    ///KDL::Jacobian Jacobian_;
    Eigen::MatrixXd Jacobian_;

    Eigen::VectorXd tau_;

    Eigen::VectorXd tau_nullspace_;

    Eigen::VectorXd F_task_;

    Eigen::VectorXd qdot_reference_;

    Eigen::VectorXd q_reference_;
    sensor_msgs::JointState left_arm_msg_;
    sensor_msgs::JointState right_arm_msg_;
    sensor_msgs::JointState torso_msg_;
    sensor_msgs::JointState head_msg_;

    //! Nullspace projection matrix
    Eigen::MatrixXd N_;

    //! Vector containing bools to indicate the previous 'active' chains (an update is provided as function of the update hook)
    //std::vector<bool> isactive_vector_;

    //! Vector containing the number of active tasks during previous loop
    //uint previous_num_active_tasks_;

    //! Sampling time
    double Ts;

    /**
     * Initialize function
     */
    bool initialize();

    /**
     * Publish reference positions
     */
    void publishReferences();

    /**
      * computeForwardKinematics
      */
    void computeForwardKinematics(KDL::Frame &FK_end_effector_pose, const std::string &end_effector_frame);

    /**
      * Forward Kinematics Solver
      */
    KDL::ChainFkSolverPos_recursive* fk_solver;
    KDL::ChainFkSolverPos_recursive* fk_solver_left;
    KDL::ChainFkSolverPos_recursive* fk_solver_right;

    /**
      * End-effector pose based on forward kinematics computation
      */
    KDL::Frame end_effector_pose_;
    KDL::Frame FK_end_effector_pose_;

    /**
     * Create the forward kinematics solvers for both chains
     */
    void createFKsolvers(const std::vector<Chain *> &chains);



};

#endif
