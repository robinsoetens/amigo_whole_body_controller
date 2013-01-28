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

    geometry_msgs::PoseStamped endEffectorPoseLeft;
    geometry_msgs::PoseStamped endEffectorPoseRight;

    Chain* chain_left_;
    Chain* chain_right_;

};

#endif // ROBOTSTATE_H
