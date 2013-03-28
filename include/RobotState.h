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

class RobotState {

public:

    //Constructor
    RobotState();

    //Destructor
    virtual ~RobotState();

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
