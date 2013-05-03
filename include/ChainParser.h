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

    static bool parse(std::vector<Chain*>& chains,
                      KDL::Tree& tree,
                      std::map<std::string, unsigned int>& joint_name_to_index,
                      std::vector<std::string>& index_to_joint_name,
                      KDL::JntArray& q_min,
                      KDL::JntArray& q_max);

    static Chain* parseChain(XmlRpc::XmlRpcValue& chain_params, const KDL::Tree& tree, urdf::Model& robot_model,
                             std::map<std::string, unsigned int>& joint_name_to_index,
                             std::vector<std::string>& index_to_joint_name, std::vector<double>& q_min, std::vector<double>& q_max);

};

#endif
