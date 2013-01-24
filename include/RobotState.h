/*!
 * \author Paul Metsemakers
 * \date January, 2013
 * \version 0.1
 */

#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

// Messages
#include <geometry_msgs/PoseStamped.h>

class RobotState {

public:

    //Constructor
    RobotState();

    //Destructor
    virtual ~RobotState();

    geometry_msgs::PoseStamped endEffectorPoseLeft;

    geometry_msgs::PoseStamped endEffectorPoseRight;

};

#endif // ROBOTSTATE_H
