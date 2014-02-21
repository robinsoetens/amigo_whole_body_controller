/*!
 * \author Janno Lunenburg
 * \date October, 2012
 * \version 0.1
 */

#ifndef ROBOTINTERFACE_H
#define ROBOTINTERFACE_H

#include <ros/ros.h>
#include <WholeBodyController.h>
#include <sensor_msgs/JointState.h>

class RobotInterface {

public:

    /**
      * Constructor
      * @param wbc Pointer to whole-body controller
      */
    RobotInterface(WholeBodyController *wbc);

    /**
      * Deconstructor
      */
    virtual ~RobotInterface();

    /**
      * Initialize function: sets publishers, subscribers, etc.
      */
    bool initialize();

    /**
      * Publishes joint references in jointstate messages
      * @param joint_refs Desired joint positions
      * @param joint_names Corresponding joint names
      */
    void publishJointReferences(const Eigen::VectorXd& joint_refs, const std::vector<std::string>& joint_names);

    /**
      * Publishes joint torques in jointstate messages
      * @param joint_torques Desired joint torques
      * @param joint_names Corresponding joint names
      */
    void publishJointTorques(const Eigen::VectorXd& joint_torques, const std::vector<std::string>& joint_names);

    /**
      * Sets base pose into whole-body controller using tf
      */
    void setAmclPose();
    
    /** Checks if all joints have been initialized */
    bool isInitialized();

protected:

    /** Pointer to whole-body controller object */
    WholeBodyController *wbc_;

    /** Subscribers to jointstate topics */
    ros::Subscriber torso_sub_, left_arm_sub_, right_arm_sub_, neck_sub_;

    /** Tf listener (required for base pose) */
    tf::TransformListener listener_;

    /** Struct containing publisher and message */
    struct JointRefPublisher {

        JointRefPublisher(const std::string& ref_topic) {
            ros::NodeHandle nh;
            pub_ = nh.advertise<sensor_msgs::JointState>(ref_topic, 10);
        }
        ros::Publisher pub_;
        sensor_msgs::JointState msg_;
    };

    /** Publishers to jointstate topics */
    JointRefPublisher *torso_pub_, *left_arm_pub_, *right_arm_pub_, *neck_pub_;

    /** Publisher to cmd_vel */
    ros::Publisher base_pub_;

    /** Maps joint name to the correct publisher */
    std::map<std::string, JointRefPublisher*> joint_name_to_pub_;
    
    /** Maps joint name to initialize bool */
    std::map<std::string, bool> initialize_map_;
    
    /** Bool indicates whether values have been received for all joints and the base */
    bool initialized_, base_initialized_;

    /**
      * Callback function for jointstate messages
      */
    void jointMeasurementCallback(const sensor_msgs::JointState::ConstPtr& msg);

};

#endif
