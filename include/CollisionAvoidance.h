/*!
 * \author Paul Metsemakers
 * \date January, 2013
 * \version 0.1
 */

#ifndef WBC_OBSTACLEAVOIDANCE_H_
#define WBC_OBSTACLEAVOIDANCE_H_

// ROS
#include "ros/ros.h"

// tf
#include <tf/transform_listener.h>

// Eigen
#include <Eigen/Core>

#include "MotionObjective.h"

/////

class CollisionAvoidance : public MotionObjective {

    struct Box {

        Box(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
            : min_(min), max_(max) {
        }

        Eigen::Vector3d min_;
        Eigen::Vector3d max_;
    };

public:

    //ToDo: make configure, start- and stophook. Components can be started/stopped in an actionlib kind of fashion

    /**
     * Constructor
     */
    CollisionAvoidance(const double Ts, const std::string& end_effector_frame, tf::TransformListener* tf_listener);

    /**
     * Deconstructor
     */
    virtual ~CollisionAvoidance();

    /**
     * Initialize function
     */
    bool initialize();


    bool isActive();

    void apply(const RobotState& robotstate);

    void visualize(const tf::Stamped<tf::Pose> &tf_end_effector_pose_MAP, const Eigen::VectorXd& wrench) const;

protected:

    //! Sampling time
    double Ts_;

    Chain* chain_;

    std::string end_effector_frame_;

    tf::TransformListener& listener_;

    std::vector<Box*> boxes_;

    ros::Publisher pub_marker_;

    geometry_msgs::PoseStamped end_effector_msg_;
    tf::Stamped<tf::Pose> tf_end_effector_pose_;
    tf::Stamped<tf::Pose> tf_end_effector_pose_MAP_;

    geometry_msgs::PoseStamped contactpoint_msg_;

    void getposeRPY(geometry_msgs::PoseStamped& pose, Eigen::Vector3d& RPY);

    void transformWench(geometry_msgs::PoseStamped& pose, Eigen::Vector3d &wrench_in, Eigen::VectorXd& wrench_out);

    void stampedPoseToKDLframe(geometry_msgs::PoseStamped& pose, KDL::Frame& frame, Eigen::Vector3d& RPY);

    void selfCollision(geometry_msgs::PoseStamped end_effector,geometry_msgs::PoseStamped contactpoint, Eigen::VectorXd& wrench_self_collision);

    void environmentCollision(tf::Stamped<tf::Pose>& tf_end_effector_pose_MAP, Eigen::VectorXd& wrench_out);


};

#endif
