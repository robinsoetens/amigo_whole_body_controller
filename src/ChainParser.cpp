#include "ChainParser.h"

using namespace std;

ChainParser::ChainParser() {

}

ChainParser::~ChainParser() {

}

bool ChainParser::parse(std::vector<Chain*>& chains, std::map<std::string, unsigned int>& joint_name_to_index, std::vector<std::string>& index_to_joint_name, KDL::JntArray& q_min, KDL::JntArray& q_max) {
    ros::NodeHandle n("~");
    std::string ns = n.getNamespace();

    /* * * * * * * * PARSE JOINT TOPICS * * * * * * * * */


    /* * * * * * * * PARSE URDF * * * * * * * * */

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

    vector<double> q_min_vec;
    vector<double> q_max_vec;

    for(int i = 0; i < chain_params.size(); ++i) {
        ROS_INFO("Going to parse chain ...");
        Chain* chain = parseChain(chain_params[i], tree, robot_model, joint_name_to_index, index_to_joint_name, q_min_vec, q_max_vec);
        ROS_INFO("Parse chain ready");
        if (chain) {
            chains.push_back(chain);
        }
    }

    q_min.resize(joint_name_to_index.size());
    q_max.resize(joint_name_to_index.size());

    for(unsigned int i = 0; i < joint_name_to_index.size(); ++i) {
        q_min(i) = q_min_vec[i];
        q_max(i) = q_max_vec[i];
    }

    ROS_INFO("Parsing done");

    return true;
}

Chain* ChainParser::parseChain(XmlRpc::XmlRpcValue& chain_description, const KDL::Tree& tree, urdf::Model& robot_model,
                               std::map<std::string, unsigned int>& joint_name_to_index,
                               std::vector<std::string>& index_to_joint_name,
                               vector<double>& q_min, vector<double>& q_max) {
    ros::NodeHandle n("~");
    std::string ns = n.getNamespace();

    cout << "A" << endl;
    cout << chain_description << endl;

    if (chain_description.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("Chain description should be a struct containing 'root' and 'tip'. (namespace: %s)", n.getNamespace().c_str());
        return 0;
    }

    cout << "B" << endl;

    XmlRpc::XmlRpcValue& v_root_link = chain_description["root"];
    if (v_root_link.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR("Chain description for does not contain 'root'. (namespace: %s)", n.getNamespace().c_str());
        return 0;
    }
    std::string root_link_name = (std::string)v_root_link;

    cout << "C" << endl;

    XmlRpc::XmlRpcValue& v_tip_link = chain_description["tip"];
    if (v_tip_link.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR("Chain description for does not contain 'tip'. (namespace: %s)", n.getNamespace().c_str());
        return 0;
    }
    std::string tip_link_name = (std::string)v_tip_link;

    cout << "D" << endl;

    Chain* chain = new Chain();

    //if (!tree.getChain(root_link_name, tip_link_name, chain->kdl_chain_)) {
    if (!tree.getChain("base", tip_link_name, chain->kdl_chain_)) {
        ROS_FATAL("Could not initialize chain object");
        return 0;
    }

    // Read joints and make index map of joints
    if (!readJoints(robot_model, root_link_name, tip_link_name, joint_name_to_index, index_to_joint_name, q_min, q_max, *chain)) {
        ROS_FATAL("Could not read information about the joints");
        return 0;
    }

    chain->jnt_to_jac_solver_ = new KDL::ChainJntToJacSolver(chain->kdl_chain_);

    return chain;
}

bool ChainParser::readJoints(urdf::Model &robot_model,
                             const std::string& root_link_name,
                             const std::string& tip_link_name,
                             std::map<std::string, unsigned int>& joint_name_to_index,
                             std::vector<std::string>& index_to_joint_name, vector<double>& q_min, vector<double>& q_max,
                             Chain& chain) {

    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_link_name);
    boost::shared_ptr<const urdf::Joint> joint;

    if (!link) {
        ROS_ERROR("Could not find link '%s'' in robot model.", tip_link_name.c_str());
        return false;
    }

    while (link && link->name != root_link_name) {

        cout << " - " << link->name << endl;

        if (!link->parent_joint) {
            ROS_ERROR("Reached end of chain and root link '%s' was not found.", root_link_name.c_str());
            return false;
        }

        joint = robot_model.getJoint(link->parent_joint->name);
        if (!joint) {
            ROS_ERROR("Could not find joint: %s", link->parent_joint->name.c_str());
            return false;
        }

        // Only count joints if they're not fixed or unknown
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            unsigned int full_joint_index = index_to_joint_name.size();

            map<string, unsigned int>::iterator it_joint = joint_name_to_index.find(joint->name);

            if (it_joint == joint_name_to_index.end()) {
                // joint not yet known, so add to map
                joint_name_to_index[joint->name] = full_joint_index;
                index_to_joint_name.push_back(joint->name);

                if (joint->type != urdf::Joint::CONTINUOUS) {
                    q_min.push_back(joint->limits->lower);
                    q_max.push_back(joint->limits->upper);


                    cout << joint->limits->lower << " - " << joint->limits->upper << endl;
                    cout << q_min[0] << " - " << q_max[0] << endl;
                }
                else {
                    ROS_WARN("Continuous joint: this is not yet nicely implemented");
                    q_min.push_back(-1000000);
                    q_max.push_back(1000000);
                }

            } else {
                full_joint_index = it_joint->second;
            }

            chain.addJoint(joint->name, link->name, full_joint_index);
        }

        link = robot_model.getLink(link->getParent()->name);
    }
    return true;
}
