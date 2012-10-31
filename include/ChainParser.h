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

    bool parse();

    bool parse(std::map<std::string, Component*>& component_map, std::map<std::string, uint>& joint_name_index_map);

    Chain* parseChain(const XmlRpc::XmlRpcValue& chain_params);

    Component* parseComponent(const XmlRpc::XmlRpcValue& component_params);

protected:

    /*
     * Function reads the number of joints in a certain chain and puts the desired values in the index_map
     *
     */
    bool readJoints(urdf::Model &robot_model, const Chain& chain, std::map<std::string, uint>& joint_name_index_map);


};

#endif
