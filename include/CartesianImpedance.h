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

class CartesianImpedance {

public:

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
    bool initialize();

protected:

    /**
     * Subscriber to target position
     */
    ros::Subscriber target_sub_;

    void callbackTarget(const geometry_msgs::PoseStamped::ConstPtr& msg);

    tf::TransformListener listener;

    geometry_msgs::PoseStamped transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped poseMsg);

    geometry_msgs::PoseStamped errorPose;

};

#endif
