#include "ConstructChain.h"

ConstructChain::ConstructChain() {

}

ConstructChain::~ConstructChain() {

}

bool ConstructChain::initialize() {

    // Get node handle
    ros::NodeHandle n("~");

    // Get URDF
    std::string urdf_xml, full_urdf_xml;
    n.param("urdf_xml",urdf_xml,std::string("robot_description"));
    n.searchParam(urdf_xml,full_urdf_xml);
    ROS_DEBUG("Reading xml file from parameter server");
    std::string result;
    if (!n.getParam(full_urdf_xml, result)) {
        ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
        error = true;
        return false;
    }

    // Load and Read Models
    if (!loadModel(result)) {
        ROS_FATAL("Could not load models!");
        error = true;
        return false;
    }
    ROS_INFO("Model loaded");

    // Construct Jacobian solver
    if (!constructJacobianSolver()) {
        ROS_FATAL("Could not construct Jacobian solver");
        error = true;
        return false;
    }

    return true;

}

bool ConstructChain::loadModel(const std::string xml) {

    //urdf::Model robot_model;
    KDL::Tree tree_;
    //tree_ = new KDL::Tree;

    /*if (!robot_model.initString(xml)) {
        ROS_FATAL("Could not initialize robot model");
        error = true;
        return false;
    }*/

    if (!kdl_parser::treeFromString(xml, tree_)) {
        ROS_ERROR("Could not initialize tree object");
        error = true;
        return false;
    }
    ROS_INFO("Tree object initialized");
    ROS_INFO("Number of joints of tree = %i",tree_.getNrOfJoints());

    /*if (!readJoints(robot_model)) {
        ROS_FATAL("Could not read information about the joints");
        error = true;
        return false;
    }
    ROS_INFO("Robot joints read");

    if (!readLinks(robot_model)) {
        ROS_FATAL("Could not read information about the links");
        error = true;
        return false;
    }
    ROS_INFO("Robot links read");*/

    treeJntToJacSolver_.reset(new KDL::TreeJntToJacSolver(tree_));

    return true;

}

/*bool Jacobian::readJoints(urdf::Model &robot_model) {
    num_joints = 0;

    return true;

}

bool Jacobian::readLinks(urdf::Model &robot_model) {

    return true;

}*/

KDL::Jacobian ConstructChain::getJacobian(const KDL::JntArray q_current_) {

    KDL::Jacobian jac;
    // Resizing here doesn't make any sense --> just for checking
    jac.resize(73);
    KDL::JntArray q_current;
    q_current.resize(73);
    std::string segmentName("grippoint_left");
    int solveResult = treeJntToJacSolver_->JntToJac(q_current, jac, segmentName);
    if (solveResult == -1) {
        ROS_WARN("Size of joint array or Jacobian not correct");
    }
    else if (solveResult == -2) {
        ROS_WARN("Segmentname not in kinematic tree");
    }

    std::cout << jac.data(1,70) << std::endl;

    return jac;

}

bool ConstructChain::constructJacobianSolver() {

    //treeJntToJacSolver_.reset(new KDL::TreeJntToJacSolver(tree_));

    return true;

}
