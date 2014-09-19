/*!
 * \author Janno Lunenburg
 * \date September, 2012
 * \version 0.1
 */

#ifndef TREEDESCRIPTION_H_
#define TREEDESCRIPTION_H_

// Vector
#include <vector>

struct component_description {
    std::string group_name;
    std::string root_name;
    std::vector<std::string> tip_names;
    int number_of_joints;
    ///std::vector<double> q;
    std::vector<std::string> joint_names;
    int start_index;
    int end_index;
    std::map<std::string, int> joint_name_map_;

};

#endif
