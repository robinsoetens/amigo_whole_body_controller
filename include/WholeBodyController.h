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

protected:

    CartesianImpedance CIleft_;
    CartesianImpedance CIright_;
    ComputeJacobian ComputeJacobian_;
    AdmittanceController AdmitCont_;
    ComputeNullspace ComputeNullspace_;
    JointLimitAvoidance JointLimitAvoidance_;
    PostureControl PostureControl_;

    ObstacleAvoidance obstacle_avoidance_;

    //! Map contains a string to describe the (root joint of the component) this concerns and a vector (is sufficient) with the current joint values
    std::map<std::string, std::vector<double> > q_current_map_;
    std::map<std::string, component_description> component_description_map_;
    std::map<std::string, uint> joint_name_index_map_;

    KDL::JntArray q_current_;
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
    std::vector<bool> isactive_vector_;

    //! Vector containing the number of active tasks during previous loop
    uint previous_num_active_tasks_;

    //! Unsigned integer containing the total number of joints
    uint num_joints_;

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
     * Subscribers to odometry and various joint positions
     */
    ros::Subscriber odom_sub_, measured_torso_position_sub_, measured_left_arm_position_sub_, measured_right_arm_position_sub_, measured_head_pan_sub_, measured_head_tilt_sub_;
    ros::Publisher torso_pub_, left_arm_pub_, right_arm_pub_, head_pub_;

    void setTopics();

    //callbackOdometry
    void callbackMeasuredTorsoPosition(const sensor_msgs::JointState::ConstPtr& msg);
    void callbackMeasuredLeftArmPosition(const sensor_msgs::JointState::ConstPtr& msg);
    void callbackMeasuredRightArmPosition(const sensor_msgs::JointState::ConstPtr& msg);
    //void callbackMeasuredHeadPan(const std_msgs::Float64::ConstPtr& msg);
    //void callbackMeasuredHeadTilt(const std_msgs::Float64::ConstPtr& msg);

};

#endif
