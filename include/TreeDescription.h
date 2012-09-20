/*!
 * \author Janno Lunenburg
 * \date September, 2012
 * \version 0.1
 */

#ifndef TREEDESCRIPTION_H_
#define TREEDESCRIPTION_H_

// ROS
#include <ros/ros.h>

// Vector
#include <vector>

struct component_description {
        std::string root_name;
        std::vector<std::string> tip_names;
        int number_of_joints;
        std::vector<double> q;
        int start_index;
        int end_index;
    };

#endif
