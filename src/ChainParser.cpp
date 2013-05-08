#include "ChainParser.h"

using namespace std;

ChainParser::ChainParser() {

}

ChainParser::~ChainParser() {

}

bool ChainParser::parse(std::vector<Chain*>& chains,
                        Tree& tree,
                        std::map<std::string,unsigned int>& joint_name_to_index,
                        std::vector<std::string>& index_to_joint_name,
                        KDL::JntArray& q_min,
                        KDL::JntArray& q_max)
{
    ros::NodeHandle n("~");
    std::string ns = n.getNamespace();

    /* * * * * * * * PARSE JOINT TOPICS * * * * * * * * */


    /* * * * * * * * PARSE URDF * * * * * * * * */

    std::string urdf_xml, full_urdf_xml;
    n.param("urdf_xml", urdf_xml,std::string("robot_description"));
    n.searchParam(urdf_xml, full_urdf_xml);
    ROS_DEBUG("Reading xml file from parameter server");
    std::string result_string;
    if (!n.getParam(full_urdf_xml, result_string)) {
        ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
        //error = true;
        return false;
    }

    urdf::Model robot_model;

    // Is this necessary?
    if (!robot_model.initString(result_string)) {
        ROS_FATAL("Could not initialize robot model");
        //error = true;
        return -1;
    }
    ROS_INFO("Robot model initialized");

    if (!kdl_parser::treeFromString(result_string, tree.kdl_tree_)) {
        ROS_ERROR("Could not initialize tree object");
        //error = true;
        return false;
    }
    ROS_INFO("Tree object initialized");

    /* * * * * * * * PARSE CHAINS * * * * * * * * */

    XmlRpc::XmlRpcValue chain_params;
    if (!n.getParam(ns + "/chain_description", chain_params)) {
        ROS_ERROR("No chain description given: %s", (ns + "/chain_description").c_str());
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

Chain* ChainParser::parseChain(XmlRpc::XmlRpcValue& chain_description, Tree tree, urdf::Model& robot_model,
                               std::map<std::string, unsigned int>& joint_name_to_index,
                               std::vector<std::string>& index_to_joint_name,
                               vector<double>& q_min, vector<double>& q_max) {
    ros::NodeHandle n("~");
    std::string ns = n.getNamespace();

    cout << chain_description << endl;

    if (chain_description.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("Chain description should be a struct containing 'root' and 'tip'. (namespace: %s)", n.getNamespace().c_str());
        return 0;
    }

    XmlRpc::XmlRpcValue& v_root_link = chain_description["root"];
    if (v_root_link.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR("Chain description for does not contain 'root'. (namespace: %s)", n.getNamespace().c_str());
        return 0;
    }
    std::string root_link_name = (std::string)v_root_link;

    XmlRpc::XmlRpcValue& v_tip_link = chain_description["tip"];
    if (v_tip_link.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR("Chain description for does not contain 'tip'. (namespace: %s)", n.getNamespace().c_str());
        return 0;
    }
    std::string tip_link_name = (std::string)v_tip_link;

    Chain* chain = new Chain();

    //if (!tree.kdl_tree.getChain(root_link_name, tip_link_name, chain->kdl_chain_)) {
    if (!tree.kdl_tree_.getChain("base", tip_link_name, chain->kdl_chain_)) {
        ROS_FATAL("Could not initialize chain object");
        return 0;
    }

    for(unsigned int i = 0; i < chain->kdl_chain_.getNrOfSegments(); ++i) {
        const KDL::Segment& segment = chain->kdl_chain_.getSegment(i);
        const KDL::Joint& joint = segment.getJoint();

        if (joint.getType() != KDL::Joint::None)
        {
            cout << "Segment: " << segment.getName() << endl;
            cout << "Joint:   " << joint.getName() << endl;

            unsigned int full_joint_index = 0;

            map<string, unsigned int>::iterator it_joint = joint_name_to_index.find(joint.getName());

            if (it_joint == joint_name_to_index.end()) {
                // joint name is not yet in map, so give it a new fresh index and add it to the map
                full_joint_index = joint_name_to_index.size();

                cout << "    new joint, gets index " << full_joint_index << endl;
                cout << "    type: " << joint.getTypeName() << endl;

                joint_name_to_index[joint.getName()] = full_joint_index;
                index_to_joint_name.push_back(joint.getName());

                cout << "    Lower limit: " << robot_model.getJoint(joint.getName())->limits->lower << endl;
                cout << "    Upper limit: " << robot_model.getJoint(joint.getName())->limits->upper << endl;

                // determine joint limits from URDF
                q_min.push_back(robot_model.getJoint(joint.getName())->limits->lower);
                q_max.push_back(robot_model.getJoint(joint.getName())->limits->upper);

            } else {
                // joint name already in the map, so look-up its index
                full_joint_index = it_joint->second;

                cout << "    existing joint, has index: " << full_joint_index << endl;
            }

            chain->addJoint(joint.getName(), segment.getName(), full_joint_index);
        }

    }

    // Read joints and make index map of joints
    /*
    if (!readJoints(robot_model, root_link_name, tip_link_name, joint_name_to_index, index_to_joint_name, q_min, q_max, *chain)) {
        ROS_FATAL("Could not read information about the joints");
        return 0;
    }
    */

    chain->chain_jnt_to_jac_solver_ = new KDL::ChainJntToJacSolver(chain->kdl_chain_);

    return chain;
}
