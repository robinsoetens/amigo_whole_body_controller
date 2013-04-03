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

// Bullet GJK Closest Point calculation
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h>
#include <Bullet-C-Api.h>

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

    void visualizeCollisionModel(btConvexShape& shape, const btTransform& transform, std::string frame_id, int id, double length, double width, double height)  const;
    //! for spheres length, width and height are equal to the radius
    //! for cylinders length and width are equal to the radius

protected:

    //! Sampling time
    double Ts_;

    Chain* chain_;

    RobotState robot_state_;

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

    void collisionModel();

    void distanceCalculation(btConvexShape& shapeA,btConvexShape& shapeB,btTransform& transformA,btTransform& transformB,btPointCollector distance_out);

    void setTransform(btTransform& transform_out, geometry_msgs::PoseStamped& fkPose, geometry_msgs::PoseStamped& fixPose);

    // Robot Model
    btConvexShape* btBaseBottom_;
    btConvexShape* btBaseMiddle_;
    btConvexShape* btBaseTop_;
    btConvexShape* btSliders_;
    btConvexShape* btTorso_;
    btConvexShape* btClavicles_;
    btConvexShape* btUpperArmLeft_;
    btConvexShape* btUpperArmRight_;
    btConvexShape* btForeArmLeft_;
    btConvexShape* btForeArmRight_;
    btConvexShape* btGripperLeft_;
    btConvexShape* btGripperRight_;
    //btConvexShape* btHead_;

    btTransform btTransformBaseBottom_;
    btTransform btTransformBaseMiddle_;
    btTransform btTransformBaseTop_;
    btTransform btTransformSliders_;
    btTransform btTransformTorso_;
    btTransform btTransformClavicles_;
    btTransform btTransformUpperArmLeft_;
    btTransform btTransformUpperArmRight_;
    btTransform btTransformForeArmLeft_;
    btTransform btTransformForeArmRight_;
    btTransform btTransformGripperLeft_;
    btTransform btTransformGripperRight_;
    //btTransform btTransformHead_;

    std::vector<btConvexShape*> shapesBody;
    std::vector<btTransform> tranformsBody;

    std::vector<btConvexShape*> shapesArmLeft;
    std::vector<btTransform> tranformsArmLeft;

    std::vector<btConvexShape*> shapesArmRight;
    std::vector<btTransform> tranformsArmRight;

    std::vector<btPointCollector> distances;


};

#endif
