#include "ComputeJacobian.h"

ComputeJacobian::ComputeJacobian() {

}

ComputeJacobian::~ComputeJacobian() {

}

bool ComputeJacobian::Initialize(std::map<std::string, component_description> *component_description_map) {

    //Load parameters from parameter server
    // Get node handle
    ros::NodeHandle n("~");

    /*if (!n.getParam("chain_array", chain_array)) {
        ROS_FATAL("Cannot load chain array from parameter server");
        return false;
    }
    ROS_INFO("Param = %s",chain_array.c_str());*/

    //TODO: Make parameters in yaml file
    chain_description_array.resize(2);
    chain_description_array[0].push_back("left_arm");
    chain_description_array[0].push_back("torso");
    chain_description_array[1].push_back("right_arm");
    chain_description_array[1].push_back("torso");

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
    for (uint i = 0; i<chain_description_array.size(); i++) {

        KDL::Chain temp_chain;
        std::string chain_name = chain_description_array[i][0];
        ///ROS_INFO("Object chain %i concerns %s", i+1, chain_name.c_str());
        std::string tip_name = (*component_description_map)[chain_name].tip_names[0];
        ///ROS_INFO("Tip name %i = %s", i+1, tip_name.c_str());
        //TODO: Make "base" variable

        if (!tree.getChain("base", tip_name, temp_chain)) {
            ROS_FATAL("Could not initialize chain object");
            return false;
        }
        //ROS_INFO("Number of joints = %i",temp_chain.getNrOfJoints());
        chain_array.push_back(temp_chain);

        // Read joints and make index map of joints
        if (!readJoints(robot_model, &(*component_description_map), chain_description_array[i])) {
            ROS_FATAL("Could not read information about the joints");
            return false;
        }

        jnt_to_jac_solver_array.push_back(new KDL::ChainJntToJacSolver(temp_chain));

    }

    return true;
}

bool ComputeJacobian::readJoints(urdf::Model &robot_model, std::map<std::string, component_description> *component_description_map, std::vector<std::string> chain_description_vector) {

    // get joint maxs and mins
    ///boost::shared_ptr<const urdf::Link> link = robot_model.getLink(chain_description_vector[0]);

    std::string tip_name = (*component_description_map)[chain_description_vector[0]].tip_names[0];
    //ROS_INFO("Tip name is %s", tip_name.c_str());
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint;

    //Loop over every 'component' in the chain
    for (uint i = 0; i < chain_description_vector.size(); i++) {

        // Integer to count the number of joints of this component
        int current_num_joints = 0;

        std::string root_name = (*component_description_map)[chain_description_vector[i]].root_name;
        //ROS_INFO("Root name = %s", root_name.c_str());
        while (link && link->name != root_name) {
            joint = robot_model.getJoint(link->parent_joint->name);
            if (!joint) {
                ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
                return false;
            }
            // Only count joints if they're not fixed or unknown
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
                current_num_joints++;
            }
            link = robot_model.getLink(link->getParent()->name);
        }

        if ((*component_description_map)[chain_description_vector[i]].number_of_joints == 0) {
            num_joints += current_num_joints;
            (*component_description_map)[chain_description_vector[i]].start_index = num_joints-current_num_joints;
            (*component_description_map)[chain_description_vector[i]].end_index = num_joints-1;
            (*component_description_map)[chain_description_vector[i]].number_of_joints = current_num_joints;
            (*component_description_map)[chain_description_vector[i]].q.resize(current_num_joints);
            ROS_INFO("Component %s gets startindex %i and endindex %i", chain_description_vector[i].c_str(), (*component_description_map)[chain_description_vector[i]].start_index, (*component_description_map)[chain_description_vector[i]].end_index);
        }

    }

    return true;
}

void ComputeJacobian::Update(std::map<std::string, component_description> component_description_map, KDL::JntArray q_current, Eigen::MatrixXd& Jacobian) {

    //ROS_INFO("Updating Jacobian matrices");

    KDL::JntArray chain_joint_array;
    KDL::Jacobian chain_jacobian;
    uint index;

    // Compute Jacobian for every chain
    for (uint i = 0; i < chain_description_array.size(); i++) {

        index = 0;
        // Fill in correct joint array
        // ToDo: RESIZING SHOULDN'T BE DONE IN REALTIME
        chain_joint_array.resize(chain_array[i].getNrOfJoints());
        chain_jacobian.resize(chain_array[i].getNrOfJoints());
        ///ROS_INFO("Number of joints of this chain = %i", chain_array[i].getNrOfJoints());

        // Fill in joints for every component in this chain
        for (uint ii = 0; ii < chain_description_array[i].size(); ii++) {

            std::string component_name = chain_description_array[i][ii];
            ///ROS_INFO("Inserting %i joint positions of %s", component_description_map[component_name].number_of_joints, component_name.c_str());
            ///ROS_INFO("Size of jointarray of %s = %i", component_name.c_str(), component_description_map[component_name].q.size());

            // For every joint within this component
            // ToDo: make this more effective

            ///ROS_INFO("First value is %f", component_description_map[component_name].q[0]);

            for (int iii = 0; iii<component_description_map[component_name].number_of_joints; iii++) {
                ///ROS_INFO("Joint %i = %f", iii, component_description_map[chain_description_array[i][ii]].q[iii]);
                chain_joint_array(index) = component_description_map[component_name].q[iii];
                index++;
            }

        }
        ///ROS_INFO("Chain %s",chain_description_array[i][0].c_str());
        ///ROS_INFO("Joint values: %f, %f, %f, %f, %f, %f, %f, %f",chain_joint_array(0),chain_joint_array(1),chain_joint_array(2),chain_joint_array(3),chain_joint_array(4),chain_joint_array(5),chain_joint_array(6),chain_joint_array(7));

        // Solve Jacobian of chain
        jnt_to_jac_solver_array[i]->JntToJac(chain_joint_array,chain_jacobian);

        uint show_column = 0;
        ROS_INFO("%i column of computed Jacobian is \n%f\n%f\n%f\n%f\n%f\n%f",show_column+1,chain_jacobian.data(0,show_column),chain_jacobian.data(1,show_column),chain_jacobian.data(2,show_column),chain_jacobian.data(3,show_column),chain_jacobian.data(4,show_column),chain_jacobian.data(5,show_column));

        // Put resulting Jacobian in correct blocks into large Jacobian
        ///Block of size (p,q), starting at (i,j)   matrix.block(i,j,p,q);
        // For every component in the chain
        uint current_index = 0;
        for (uint iiii = 0; iiii < chain_description_array[i].size(); iiii++) {
            std::string component_name = chain_description_array[i][iiii];
            ///ROS_INFO("Component %s inserted in chain array", component_name.c_str());

            ///ROS_INFO("Indexes of large Jacobian are %i, %i, %i, %i",6*i,component_description_map[component_name].start_index,6,component_description_map[component_name].number_of_joints);
            ///ROS_INFO("Indexes of small Jacobian are %i, %i, %i, %i",0,current_index,6,component_description_map[component_name].number_of_joints);

            ///ROS_INFO("Chain Jacobian is of size %i x %i",chain_jacobian.data.rows(),chain_jacobian.data.cols());
            ///Eigen::MatrixXd test(6,7);
            ///Jacobian.block(0,0,6,7) = test;

            ///Eigen::MatrixXd test(6,component_description_map[component_name].number_of_joints);
            ///Jacobian.block(0,0,6,component_description_map[component_name].number_of_joints) = test;

            ///Eigen::MatrixXd test(6,component_description_map[component_name].number_of_joints);
            ///Jacobian.block(0,current_index,6,component_description_map[component_name].number_of_joints) = test;

            ///Eigen::MatrixXd test(6,component_description_map[component_name].number_of_joints);
            ///Jacobian.block(6*i,current_index,6,component_description_map[component_name].number_of_joints) = test;

            ///Eigen::MatrixXd test(6,component_description_map[component_name].number_of_joints);
            ///Jacobian.block(6*i,component_description_map[component_name].start_index,6,component_description_map[component_name].number_of_joints) = test;

            Jacobian.block(6*i,component_description_map[component_name].start_index,6,component_description_map[component_name].number_of_joints) =
                    chain_jacobian.data.block(0,current_index,6,component_description_map[component_name].number_of_joints);
            current_index += component_description_map[component_name].number_of_joints;

            ///ROS_INFO("Size of Jacobian is %i x %i",Jacobian.data.rows(),Jacobian.data.cols());
        }
    }
    ROS_INFO("Eighth column of computed Jacobian is \n%f\n%f\n%f\n%f\n%f\n%f",Jacobian(0,7),Jacobian(1,7),Jacobian(2,7),Jacobian(3,7),Jacobian(4,7),Jacobian(5,7));
}
