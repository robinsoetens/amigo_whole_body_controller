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
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// KDL
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

// Bullet
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

// FCL
#include <fcl/collision.h>

class RobotState
{

public:

    //Constructor
    RobotState();

    //Destructor
    virtual ~RobotState();

    // Map containing segment names and corresponding poses (in map frame)
    std::map<std::string, KDL::Frame> fk_poses_;

    // Collision Model
    // ToDo: why have both name_collision_body and frame_id? --> Convenient when adding objects to gripper
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
        std::string frame_id;
        KDL::Frame fk_pose;
        KDL::Frame fix_pose;
        btTransform bt_transform;
        btConvexShape* bt_shape;
        fcl::Transform3f fcl_transform;
        fcl::CollisionGeometry* fcl_shape;

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

            frame_id = static_cast<std::string>(corrTransform["frame_id"]);
            fix_pose.p.x(origin["x"]);
            fix_pose.p.y(origin["y"]);
            fix_pose.p.z(origin["z"]);
            fix_pose.M.Quaternion(orientation["x"],
                                  orientation["y"],
                                  orientation["z"],
                                  orientation["w"]);
        }
    };

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

    Tree tree_;
    KDL::Frame amcl_pose_;

    /**
      * Forward Kinematics Solver
      */
    KDL::TreeFkSolverPos_recursive* fk_solver_;

    /**
      * Store all FK solutions in a map
      */
    void collectFKSolutions();

    /** Updates poses of all collision bodies */
    void updateCollisionBodyPoses();

    /**
      * Returns the current FK solution
      */
    void KDLFrameToStampedPose(const KDL::Frame &FK_pose, geometry_msgs::PoseStamped &pose);

    int getNrOfSegment(KDL::Chain kdl_chain_, const std::string& segment_name);

    /**
      * Returns the KDL frame of the tip frame
      * @param tip_frame
      */
    KDL::Frame getFK(const std::string& tip_frame);

    void setAmclPose(KDL::Frame& amcl_pose);

    /** Returns number of joints */
    unsigned int getNrJoints();

};

#endif // ROBOTSTATE_H


