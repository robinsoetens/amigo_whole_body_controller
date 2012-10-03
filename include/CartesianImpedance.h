/*!
 * \author Janno Lunenburg
 * \date September, 2012
 * \version 0.1
 */

#ifndef CARTESIANIMPEDANCE_H_
#define CARTESIANIMPEDANCE_H_

// ROS
#include "ros/ros.h"

// Messages
#include <geometry_msgs/PoseStamped.h>

// tf
#include <tf/transform_listener.h>

// Eigen
#include <Eigen/Core>

class CartesianImpedance {

public:

    //ToDo: make configure, start- and stophook. Components can be started/stopped in an actionlib kind of fashion

    /**
     * Constructor
     */
    CartesianImpedance();

    /**
     * Deconstructor
     */
    ~CartesianImpedance();

    /**
     * Initialize function
     */
    bool initialize(std::string);

    /**
     * Update
     */
    void update(Eigen::MatrixXd, Eigen::VectorXd&);

protected:

    /**
     * Subscriber to target position
     */
    ros::Subscriber target_sub_;

    void callbackTarget(const geometry_msgs::PoseStamped::ConstPtr& msg);

    tf::TransformListener listener;

    geometry_msgs::PoseStamped transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped poseMsg);

    geometry_msgs::PoseStamped errorPose;
    Eigen::VectorXd error_vector_;

    // Cartesian Impedance matrix
    Eigen::MatrixXd K_;

    std::string end_effector_frame_;

};

#endif
