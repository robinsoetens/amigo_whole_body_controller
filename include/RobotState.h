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

class RobotState {

public:

    //Constructor
    RobotState();

    //Destructor
    virtual ~RobotState();

    struct CollisionShape {
        std::string name_collision_part;
        struct Shape {
            std::string shape_type;
            struct Dimensions {
                double x;
                double y;
                double z;
            };

            void fromXmlRpc(XmlRpc::XmlRpcValue& value)
            {
                shape_type = static_cast<std::string>(value["shape_type"]);
                Dimensions.x = value["dimensions"];
                Dimensions.y = value["dimensions"];
                Dimensions.z = value["dimensions"];
            }
        };
        geometry_msgs::PoseStamped frame_pose;
        struct CorrectionTransform {
            std::string frame_id;
            struct Origin {
                double x;
                double y;
                double z;
            };
            struct Orientation {
                double x;
                double y;
                double z;
                double w;
            };
        };
    };

    std::vector< CollisionShape > groupBody;
    std::vector< CollisionShape > groupArmLeft;
    std::vector< CollisionShape > groupArmRight;

    std::vector< std::vector<CollisionShape> >  Robot;

    geometry_msgs::PoseStamped poseBase_;
    geometry_msgs::PoseStamped poseSliders_;
    geometry_msgs::PoseStamped poseTorso_;
    geometry_msgs::PoseStamped poseClavicles_;
    geometry_msgs::PoseStamped poseUpperArmLeft_;
    geometry_msgs::PoseStamped poseUpperArmRight_;
    geometry_msgs::PoseStamped poseForeArmLeft_;
    geometry_msgs::PoseStamped poseForeArmRight_;
    geometry_msgs::PoseStamped poseGrippointLeft_;
    geometry_msgs::PoseStamped poseGrippointRight_;
    geometry_msgs::PoseStamped poseHead_;

    Chain* chain_left_;
    Chain* chain_right_;

};

#endif // ROBOTSTATE_H


