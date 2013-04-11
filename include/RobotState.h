/*!
 * \author Paul Metsemakers
 * \date January, 2013
 * \version 0.1
 */

#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

// Messages
#include <geometry_msgs/PoseStamped.h>

// KDL
#include "Chain.h"
#include <XmlRpc.h>

// Bullet
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>


class RobotState
{

public:

    //Constructor
    RobotState();

    //Destructor
    virtual ~RobotState();

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
        std::string chain_side;
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

            chain_side = static_cast<std::string>(corrTransform["chain_side"]);

            fix_pose.header.frame_id = static_cast<std::string>(corrTransform["frame_id"]);
            fix_pose.pose.position.x = origin["x"];
            fix_pose.pose.position.y = origin["y"];
            fix_pose.pose.position.z = origin["z"];
            fix_pose.pose.orientation.x = orientation["x"];
            fix_pose.pose.orientation.y = orientation["y"];
            fix_pose.pose.orientation.z = orientation["z"];
            fix_pose.pose.orientation.w = orientation["w"];

            /*
            std::cout << "name = " << name_collision_body << std::endl;
            std::cout << "frame_id = " << fix_pose.header.frame_id << std::endl;
            std::cout << "x = " << fix_pose.pose.position.x << std::endl;
            std::cout << "y = " << fix_pose.pose.position.y << std::endl;
            std::cout << "z = " << fix_pose.pose.position.z << std::endl;
            std::cout << "X = " << fix_pose.pose.orientation.x << std::endl;
            std::cout << "Y = " << fix_pose.pose.orientation.y << std::endl;
            std::cout << "Z = " << fix_pose.pose.orientation.z << std::endl;
            std::cout << "W = " << fix_pose.pose.orientation.w << std::endl;
            */
        }
    } collision_body;

    struct Robot {
        std::vector< std::vector<CollisionBody> > groups;
    } robot_;


    struct Distance {
        std::string frame_id;
        btPointCollector bt_distance;
    };


    geometry_msgs::PoseStamped poseGrippointLeft_;
    geometry_msgs::PoseStamped poseGrippointRight_;

    Chain* chain_left_;
    Chain* chain_right_;

};

#endif // ROBOTSTATE_H


