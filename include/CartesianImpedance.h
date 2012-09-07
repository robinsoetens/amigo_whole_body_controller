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

protected:

    /**
     * Initialize function
     */
    bool initialize();

    /**
     * Subscribers to odometry and various joint positions
     */
    ros::Subscriber target_sub_;

    void callbackTarget(const geometry_msgs::PoseStamped::ConstPtr& msg);

    tf::TransformListener listener;
    geometry_msgs::PoseStamped transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped poseMsg);
    //void transformPose(const std::string &target_frame, const Stamped< tf::Pose > &stamped_in, Stamped< tf::Pose > &stamped_out);

    geometry_msgs::PoseStamped errorPose;

};

#endif
