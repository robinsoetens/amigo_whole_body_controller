#ifndef WBC_CHAIN_PARSER_H_
#define WBC_CHAIN_PARSER_H_

// ROS
#include <ros/ros.h>

// Construct Chain
#include "ConstructChain.h"

#include "TreeDescription.h"

// KDL
#include <kdl/chainjnttojacsolver.hpp>

// Vector and map
#include <vector>
#include <map>

#include "Chain.h"
#include "Component.h"

class ChainParser {

public:

    ChainParser();

    virtual ~ChainParser();

    static bool parse(std::vector<Chain*>& chains, std::map<std::string, unsigned int>& joint_name_to_index, std::vector<std::string>& index_to_joint_name);

    static Chain* parseChain(const XmlRpc::XmlRpcValue& chain_params);

    static Component* parseComponent(const XmlRpc::XmlRpcValue& component_params);

protected:

    /*
     * Function reads the number of joints in a certain chain and puts the desired values in the index_map
     *
     */
    static bool readJoints(urdf::Model &robot_model, const std::string &root_link_name, const std::string &tip_link_name);


};

#endif
