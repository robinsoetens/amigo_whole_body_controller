/*!
 * \author Paul Metsemakers
 * \date January, 2013
 * \version 0.1
 */

#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <map>
#include "Chain.h"
#include "Tree.h"
#include <XmlRpc.h>

// Messages
#include <geometry_msgs/PoseStamped.h>

// KDL
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

// Bullet
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

// tf
#include <tf/transform_listener.h>


class RobotState
{

public:

    //Constructor
    RobotState();

    //Destructor
    virtual ~RobotState();

    std::map<std::string, geometry_msgs::PoseStamped> fk_poses_;

    // Collision Model
    struct CollisionBody
    {
        std::string name_collision_body;
        struct CollisionShape
        {
            std::string shape_type;
            struct Dimensions
            {
                double x;
                double y;
                double z;
            } dimensions;
        } collision_shape;
        geometry_msgs::PoseStamped fk_pose;
        geometry_msgs::PoseStamped fix_pose;
        btTransform bt_transform;
        btConvexShape* bt_shape;

        void fromXmlRpc(XmlRpc::XmlRpcValue& value)
        {
            name_collision_body = static_cast<std::string>(value["name"]);

            // COLLISION SHAPE
            XmlRpc::XmlRpcValue collShape = value["shape"];
            collision_shape.shape_type =  static_cast<std::string>(collShape["shape_type"]);

            XmlRpc::XmlRpcValue dim = collShape["dimensions"];
            collision_shape.dimensions.x = dim["x"];
            collision_shape.dimensions.y = dim["y"];
            collision_shape.dimensions.z = dim["z"];

            // CORRECTION TRANSFORM
            XmlRpc::XmlRpcValue corrTransform = value["transform"];
            XmlRpc::XmlRpcValue origin = corrTransform["origin"];
            XmlRpc::XmlRpcValue orientation = corrTransform["orientation"];

            fix_pose.header.frame_id = static_cast<std::string>(corrTransform["frame_id"]);
            fix_pose.pose.position.x = origin["x"];
            fix_pose.pose.position.y = origin["y"];
            fix_pose.pose.position.z = origin["z"];
            fix_pose.pose.orientation.x = orientation["x"];
            fix_pose.pose.orientation.y = orientation["y"];
            fix_pose.pose.orientation.z = orientation["z"];
            fix_pose.pose.orientation.w = orientation["w"];
        }
    } collision_body;

    struct Exclusion
    {
        std::string name_body_A;
        std::string name_body_B;

        void fromXmlRpc(XmlRpc::XmlRpcValue& value)
        {
            name_body_A = static_cast<std::string>(value["NameBodyA"]);
            name_body_B = static_cast<std::string>(value["NameBodyB"]);
        }

    } exclusion;

    struct Robot
    {
        std::vector< std::vector<CollisionBody> > groups;
    } robot_;

    struct ExclusionChecks
    {
        std::vector<Exclusion> checks;
    } exclusion_checks;

    geometry_msgs::PoseStamped poseGrippointLeft_;
    geometry_msgs::PoseStamped poseGrippointRight_;

    Tree tree_;
    Chain* chain_left_;
    Chain* chain_right_;
    std::vector<Chain*> chains_;

    /**
      * Forward Kinematics Solver
      */
    KDL::TreeFkSolverPos_recursive* fk_solver_;

    /**
      * Store all FK solutions in a map
      */
    void collectFKSolutions(tf::TransformListener &listener);

    /**
      * Returns the current FK solution
      *
      */
    void KDLFrameToStampedPose(const KDL::Frame &FK_pose, geometry_msgs::PoseStamped &pose);

    int getNrOfSegment(KDL::Chain kdl_chain_, const std::string& segment_name);

    /**
      * Returns the KDL frame of the tip frame
      * @param tip_frame
      */
    KDL::Frame getFK(const std::string& tip_frame);

};

#endif // ROBOTSTATE_H


