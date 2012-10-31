#include "ChainParser.h"

ChainParser::ChainParser() {

}

ChainParser::~ChainParser() {

}


Chain* ChainParser::parseChain(const XmlRpc::XmlRpcValue& chain_params) {

}

Component* ChainParser::parseComponent(const XmlRpc::XmlRpcValue& component_struct) {
    if (!component_struct.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("Component description should be an struct. (namespace: %s)", n.getNamespace().c_str());
        return 0;
    }

    if (component_struct.size() != 1) {
        ROS_ERROR("Malformed component description. (namespace: %s)", n.getNamespace().c_str());
        return 0;
    }

    std::string component_name = component_struct.begin()->first;
    Component* component = new Component(component_name);

    XmlRpc::XmlRpcValue& component_param = component_struct.begin()->second;

    XmlRpc::XmlRpcValue& v_root_name = component_param["root_name"];
    if (v_root_name.getType() == XmlRpc::XmlRpcValue::TypeString) {
        component->setRootLinkName((std::string)v_root_name);
    } else {
        ROS_ERROR("Component description for '%s' does not contain 'root_name'. (namespace: %s)", component->getName().c_str(), n.getNamespace().c_str());
        return 0;
    }

    XmlRpc::XmlRpcValue& v_leaf_names = component_param["leaf_names"];
    if (v_leaf_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Component description for '%s' doesn not contain 'leaf_names'. (namespace: %s)", component->getName().c_str(), n.getNamespace().c_str());
        return 0;
    }

    for(int j = 0 ; j < v_leaf_names.size(); ++j) {
        component->addLeafLinkName((std::string)v_leaf_names[j]);
    }

    XmlRpc::XmlRpcValue& v_meas_topic = component_param["measurement_topic"];
    if (v_meas_topic.getType() == XmlRpc::XmlRpcValue::TypeString) {
        component->setMeasurementTopic((std::string)v_meas_topic);
    } else {
        ROS_ERROR("Component description for '%s' does not contain 'measurement_topic'. (namespace: %s)", component->getName().c_str(), n.getNamespace().c_str());
        return 0;
    }

    XmlRpc::XmlRpcValue& v_ref_topic = component_param["reference_topic"];
    if (v_meas_topic.getType() == XmlRpc::XmlRpcValue::TypeString) {
        component->setReferenceTopic((std::string)v_ref_topic);
    } else {
        ROS_ERROR("Component description for '%s' does not contain 'reference_topic'. (namespace: %s)", component->getName().c_str(), n.getNamespace().c_str());
        return 0;
    }

    return component;
}

bool ChainParser::parse() {
    ros::NodeHandle n("~");
    std::string ns = n.getNamespace();

    unsigned int num_joints = 0;
    std::map<std::string, Component*> component_map;

    XmlRpc::XmlRpcValue component_params;
    if (!n.getParam(ns + "/component_description", component_params)) {
        ROS_ERROR("No component description given. (namespace: %s)", n.getNamespace().c_str());
        return false;
    }

    if (component_params.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Component description list should be an array.  (namespace: %s)", n.getNamespace().c_str());
        return false;
    }

    for(int i = 0; i < component_params.size(); ++i) {
        Component* component = parseComponent(component_params[i]);
        if (component) {
            component_map[component->getName()] = component;
        }
    }

    /* * * * * * * * PARSE CHAINS * * * * * * * * */

    XmlRpc::XmlRpcValue chain_params;
    if (!n.getParam(ns + "/chain_description", chain_params)) {
        ROS_ERROR("No chain description given. (namespace: %s)", n.getNamespace().c_str());
        return false;
    }

    if (chain_params.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Chain description list should be an array.  (namespace: %s)", n.getNamespace().c_str());
        return false;
    }

    //chain_description_array.resize(chain_params.size());
    chains_.resize(chain_params.size());
    for(int i = 0; i < chain_params.size(); ++i) {
        XmlRpc::XmlRpcValue& chain_description = chain_params[i];
        if (!chain_description.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("Chain description should be an array. (namespace: %s)", n.getNamespace().c_str());
            return false;
        }

        printf("Chain size: %i\n", chain_description.size());

        for(int j = 0; j < chain_description.size(); ++j) {
            if (chain_description[j].getType() == XmlRpc::XmlRpcValue::TypeString) {
                std::string component_name = (std::string)chain_description[j];
                std::map<std::string, Component*>::iteratot it_comp = component_map.find(component_name);
                if (it_comp != component_map.end()) {
                    chains_[i].addComponent(it_comp->second);
                } else {
                    ROS_ERROR("Invalid chain description. (namespace: %s)", n.getNamespace().c_str());
                }
            }
        }
    }
}

void ChainParser::parse(std::map<std::string, Component*>& component_map, std::map<std::string, uint>& joint_name_index_map) {

    // Get URDF
    std::string urdf_xml, full_urdf_xml;
    n.param("urdf_xml",urdf_xml,std::string("robot_description"));
    n.searchParam(urdf_xml,full_urdf_xml);
    ROS_DEBUG("Reading xml file from parameter server");
    std::string result_string;
    if (!n.getParam(full_urdf_xml, result_string)) {
        ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
        //error = true;
        return false;
    }

    urdf::Model robot_model;
    KDL::Tree tree;

    // Is this necessary?
    if (!robot_model.initString(result_string)) {
        ROS_FATAL("Could not initialize robot model");
        //error = true;
        return -1;
    }
    ROS_INFO("Robot model initialized");

    if (!kdl_parser::treeFromString(result_string, tree)) {
        ROS_ERROR("Could not initialize tree object");
        //error = true;
        return false;
    }
    ROS_INFO("Tree object initialized");
    //(Display results)

    // For every entry in array:
    //          Make kinematic chain
    //          Resize joint arrays
    //          Do bookkeeping

    // Resize jnt_to_jac_solver_array (OBSOLETE IF ARRAY IS USED INSTEAD OF VECTOR)
    //jnt_to_jac_solver_array.resize(2);

    ///ROS_INFO("Chain description array %s", chain_description_array[0][0].c_str());
    ///ROS_INFO("First tooltip = %s", component_description_map[chain_description_array[0][0].c_str()].tip_names[0].c_str());

    // Number of joints starts with zero, will increase when adding chain objects. Eventually, everything will be resized
    num_joints = 0;
    for (uint i = 0; i < chains_.size(); i++) {

        std::string chain_name = chains_[i].components_[0]->name_;
        ///ROS_INFO("Object chain %i concerns %s", i+1, chain_name.c_str());
        std::string tip_name = component_map[chain_name].tip_names[0];
        ///ROS_INFO("Tip name %i = %s", i+1, tip_name.c_str());
        //TODO: Make "base" variable

        if (!tree.getChain("base", tip_name, chains_[i].kdl_chain_)) {
            ROS_FATAL("Could not initialize chain object");
            return false;
        }
        //ROS_INFO("Number of joints = %i", chains_[i].kdl_chain_.getNrOfJoints());

        // Read joints and make index map of joints
        if (!readJoints(robot_model, component_map, chains_[i], joint_name_index_map)) {
            ROS_FATAL("Could not read information about the joints");
            return false;
        }

        chains_[i].jnt_to_jac_solver_ = new KDL::ChainJntToJacSolver(chains_[i].kdl_chain_);

    }

    //for (uint i = 0; i < num_joints; i++) ROS_INFO("Joint limits joint %i are %f %f",i,q_min_[i],q_max_[i]);

    num_active_chains_ = 0;

    return true;
}

bool ChainParser::readJoints(urdf::Model &robot_model, const Chain& chain, std::map<std::string, uint>& joint_name_index_map) {

    // get joint maxs and mins
    ///boost::shared_ptr<const urdf::Link> link = robot_model.getLink(chain_description_vector[0]);

    //std::string tip_name = component_description_map[chain_description_vector[0]].tip_names[0];
    std::string tip_name = chain.components_[0]->tip_names_[0];
    //ROS_INFO("Tip name is %s", tip_name.c_str());
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint;

    if (!link) {
        ROS_ERROR("Could not get link for joint %s\n", tip_name.c_str());
        return false;
    }

    //Loop over every 'component' in the chain
    for (uint i = 0; i < chain_description_vector.size(); i++) {

        // Integer to count the number of joints of this component
        int current_num_joints = 0;

        // Vector containing joint limits of this component
        std::vector<double> current_temp_joint_min, current_temp_joint_max;

        std::string root_name = component_description_map[chain_description_vector[i]].root_name;

        // Vector containing the joint names
        std::vector<std::string> current_joint_names;

        //ROS_INFO("Root name = %s", root_name.c_str());
        while (link && link->name != root_name) {

            if (!link->parent_joint) {
                ROS_ERROR("Reached end of chain and link '%s' was not found.", root_name.c_str());
                return false;
            }

            joint = robot_model.getJoint(link->parent_joint->name);
            if (!joint) {
                ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());

                ROS_INFO("ComputeJacobian::readJoints - FALSE");
                return false;
            }

            // Only count joints if they're not fixed or unknown
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
                current_num_joints++;
                current_joint_names.push_back(joint->name);
                if (joint->type != urdf::Joint::CONTINUOUS) {
                    current_temp_joint_min.push_back(joint->limits->lower);
                    current_temp_joint_max.push_back(joint->limits->upper);
                }
                else {
                    ROS_WARN("This is not yet nicely implemented");
                    current_temp_joint_min.push_back(-1000000);
                    current_temp_joint_max.push_back(1000000);
                }
            }

            link = robot_model.getLink(link->getParent()->name);
        }

        // Only add info to component description map if this has not been done yet
        if (component_description_map[chain_description_vector[i]].number_of_joints == 0) {
            num_joints += current_num_joints;
            component_description_map[chain_description_vector[i]].start_index = num_joints-current_num_joints;
            component_description_map[chain_description_vector[i]].end_index = num_joints-1;
            component_description_map[chain_description_vector[i]].number_of_joints = current_num_joints;
            ///component_description_map[chain_description_vector[i]].q.resize(current_num_joints);
            ROS_INFO("Component %s gets startindex %i and endindex %i", chain_description_vector[i].c_str(), component_description_map[chain_description_vector[i]].start_index, component_description_map[chain_description_vector[i]].end_index);

            for (int k = 0; k < current_num_joints; k++) {
                //temp_joint_min.push_back(current_temp_joint_min[i]);
                //temp_joint_max.push_back(current_temp_joint_max[i]);
                // The joints are read from tip to root but appear in the joint vectors from root to tip so have to be 'used' in reverse order.
                std::string joint_name(current_joint_names[current_num_joints-1-k]);
                component_description_map[chain_description_vector[i]].joint_names.push_back(current_joint_names[current_num_joints-1-k]);
                q_min_.push_back(current_temp_joint_min[current_num_joints-1-k]);
                q_max_.push_back(current_temp_joint_max[current_num_joints-1-k]);
                ///ROS_INFO("Insert joint %s in map",joint_name.c_str());
                joint_name_index_map[joint_name] = k+num_joints-current_num_joints;
            }

        }

    }

    return true;

}
