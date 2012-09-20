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
#include <amigo_msgs/arm_joints.h>

// WholeBodyController
#include "CartesianImpedance.h"
#include "ComputeJacobian.h"
//#include "TreeDescription.h"

class WholeBodyController {

public:

    /**
     * Constructor
     */
    WholeBodyController();

    /**
     * Deconstructor
     */
    ~WholeBodyController();

    /*
     * Updatehook
     */
    bool update();

protected:

    CartesianImpedance CIleft_;
    ComputeJacobian ComputeJacobian_;

    //! Map contains a string to describe the (root joint of the component) this concerns and a vector (is sufficient) with the current joint values
    std::map<std::string, std::vector<double> > q_current_map_;
    std::map<std::string, component_description> component_description_map_;

    KDL::JntArray q_current_;
    KDL::Jacobian Jacobian_;

    /**
     * Initialize function
     */
    bool initialize();

    /**
     * Subscribers to odometry and various joint positions
     */
    ros::Subscriber odom_sub_, measured_torso_position_sub_, measured_left_arm_position_sub_, measured_right_arm_position_sub_, measured_head_pan_sub_, measured_head_tilt_sub_;

    void setTopics();

    //callbackOdometry
    void callbackMeasuredTorsoPosition(const std_msgs::Float64::ConstPtr& msg);
    void callbackMeasuredLeftArmPosition(const amigo_msgs::arm_joints::ConstPtr& msg);
    void callbackMeasuredRightArmPosition(const amigo_msgs::arm_joints::ConstPtr& msg);
    void callbackMeasuredHeadPan(const std_msgs::Float64::ConstPtr& msg);
    void callbackMeasuredHeadTilt(const std_msgs::Float64::ConstPtr& msg);



};

#endif
