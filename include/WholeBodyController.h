/*!
 * \author Janno Lunenburg
 * \date September, 2012
 * \version 0.1
 */

#ifndef WHOLEBODYCONTROLLER_H_
#define WHOLEBODYCONTROLLER_H_

// ROS
//#include "ros/ros.h"

//#include <Eigen/Core>

// Messages
//#include <std_msgs/Float64.h>
//#include <sensor_msgs/JointState.h>

// WholeBodyController
#include "CartesianImpedance.h"
#include "ComputeJacobian.h"
#include "AdmittanceController.h"
#include "ComputeNullspace.h"
#include "JointLimitAvoidance.h"
#include "PostureControl.h"
#include "CollisionAvoidance.h"
#include "amigo_whole_body_controller/Tracing.hpp"
//#include "TreeDescription.h"

// Vector
//#include <vector>

class WholeBodyController {

public:

    /**
     * Constructor
     */
    WholeBodyController(const double Ts);

    /**
     * Deconstructor
     */
    ~WholeBodyController();

    /**
     * Updatehook
     * @param q_reference: Eigen vector with reference joint positions
     * @param qdot_reference: Eigen vector with reference joint velocities
     */
    bool update(Eigen::VectorXd &q_reference, Eigen::VectorXd &qdot_reference);

    /**
      * Sets current joint position in whole-body controller object
      * @param joint_name: Joint name
      * @param pos: Current joint position
      */
    void setMeasuredJointPosition(const std::string& joint_name, double pos);

    /**
      * Returns the current joint position
      * @param joint_name Joint name
      * @return Joint position
      */
    double getJointPosition(const std::string& joint_name) const;

    /**
      * Sets desired joint position
      * @param joint_name Joint name
      * @param reference Desired joint position
      */
    bool setDesiredJointPosition(const std::string& joint_name, double reference);

    /**
      * Returns eigen vector with joint reference positions
      */
    const Eigen::VectorXd& getJointReferences() const;

    /**
      * Returns eigen vector with joint reference velocities
      */
    const Eigen::VectorXd& getJointTorques() const;

    /**
      * Returns vector with joint names
      * @return Vector with joint names
      */
    const std::vector<std::string>& getJointNames() const;

    /**
      * Adds motion objective to the whole-body controller
      * @param motionobjective: pointer to the motion objective that is added
      */
    bool addMotionObjective(MotionObjective* motionobjective);

    /**
      * Removes motion objective from the whole-body controller
      * @param motionobjective: pointer to the motionobjective to remove
      */
    bool removeMotionObjective(MotionObjective* motionobjective);

    /**
      * Returns the current "cost", i.e., the absolute value of the torque of every single plugin
      */
    double getCost();

    RobotState robot_state_;
    
    /**
      * @return Returns a map from joint names to indexes of the various Eigen vectors and KDL JntArrays
      */
    std::map<std::string, unsigned int> getJointNameToIndex();

    /**
      * Returns a vector with all motion objectives of the type CartesianImpedance with the tip and root frame as asked
      * @param tip_frame: end-effector frame of the motion objective
      * @param root_frame: root frame of the motion objective
      */
    std::vector<MotionObjective*> getCartesianImpedances(const std::string& tip_frame, const std::string& root_frame);

protected:

    /** Maps joint names to vector indices */
    std::map<std::string, unsigned int> joint_name_to_index_;

    /** Maps indices to joint names */
    std::vector<std::string> index_to_joint_name_;

    /** Joint array containing the current joint positions */
    KDL::JntArray q_current_;

    //! Unsigned integer containing the total number of joints
    uint num_joints_;

    /** Vector containing pointers to the various motion objectives */
    std::vector<MotionObjective*> motionobjectives_;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    /** Admittance controller: integrates desired torques to velocities and positions
      using M \ddot q + D \dot q = tau */
    AdmittanceController AdmitCont_;

    /** Computes the null space of the combined Jacobian (Cartesian impedances + collision avoidance) */
    ComputeNullspace ComputeNullspace_;

    /** Joint limit avoidance: pushes joints away from their joint limits if they come to close
      Is projected into the task null space to avoid interference with the main tasks */
    JointLimitAvoidance JointLimitAvoidance_;

    /** Keeps the robot in a desired posture. Is also used to handle joint goals
      Is projected into the task null space to avoid interference with the main tasks */
    PostureControl PostureControl_;

    /** Matrix containing the combined Jacobian (Cartesian impedances + collision avoidance) */
    Eigen::MatrixXd Jacobian_;

    /** Vector containing the desired joint torques */
    Eigen::VectorXd tau_;

    /** Vector containing the joint torques that are projected in the null space */
    Eigen::VectorXd tau_nullspace_;

    /** Vector containing the elements of all wrenches */
    Eigen::VectorXd F_task_;

    /** Vector containing the desired joint velocities */
    Eigen::VectorXd qdot_reference_;

    /** Vector containing the desired joint positions */
    Eigen::VectorXd q_reference_;

    //! Nullspace projection matrix
    Eigen::MatrixXd N_;

    /** Initialize function */
    bool initialize(const double Ts);

    /** Subtracts information from the parameter files */
    void loadParameterFiles(RobotState &robot_state);

    /** Tracing object */
    Tracing tracer_;

};

#endif
