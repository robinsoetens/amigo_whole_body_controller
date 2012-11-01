#include "WholeBodyController.h"
#include "ChainParser.h"

using namespace std;

WholeBodyController::WholeBodyController() {
    initialize();
}

WholeBodyController::~WholeBodyController() {
    ROS_INFO("Shutting down whole body controller");
}

bool WholeBodyController::initialize() {
    ros::NodeHandle n("~");
    std::string ns = n.getNamespace();
    //ROS_INFO("Nodehandle %s",n.getNamespace().c_str());
    ROS_INFO("Initializing whole body controller");

    // ToDo: Parameterize
    Ts = 0.1;

    ChainParser::parse(chains_, joint_name_to_index_, index_to_joint_name_);
    num_joints_ = joint_name_to_index_.size();

    q_current_.resize(num_joints_);
    for(unsigned int i = 0; i < q_current_.rows(); ++i) {
        q_current_(i) = 0;
    }

    // Read parameters
    std::vector<double> JLA_gain(num_joints_);      // Gain for the joint limit avoidance
    std::vector<double> JLA_workspace(num_joints_); // Workspace: joint gets 'pushed' back when it's outside of this part of the center of the workspace

    std::vector<double> posture_q0(num_joints_);
    std::vector<double> posture_gain(num_joints_);

    std::vector<double> admittance_mass(num_joints_);
    std::vector<double> admittance_damping(num_joints_);
    for (std::map<std::string, uint>::iterator iter = joint_name_index_map_.begin(); iter != joint_name_index_map_.end(); ++iter) {
        n.param<double> (ns+"/joint_limit_avoidance/gain/"+iter->first, JLA_gain[iter->second], 1.0);
        n.param<double> (ns+"/joint_limit_avoidance/workspace/"+iter->first, JLA_workspace[iter->second], 0.9);
        n.param<double> (ns+"/posture_control/home_position/"+iter->first, posture_q0[iter->second], 0);
        n.param<double> (ns+"/posture_control/gain/"+iter->first, posture_gain[iter->second], 1);
        n.param<double> (ns+"/admittance_control/mass/"+iter->first, admittance_mass[iter->second], 10);
        n.param<double> (ns+"/admittance_control/damping/"+iter->first, admittance_damping[iter->second], 10);
        ROS_INFO("Damping joint %s = %f",iter->first.c_str(),admittance_damping[iter->second]);
    }

    // Initialize admittance controller
    AdmitCont_.initialize(ComputeJacobian_.q_min_, ComputeJacobian_.q_max_, admittance_mass, admittance_damping);

    // Initialize nullspace calculator
    // ToDo: Make this variable
    Eigen::MatrixXd A;
    /*A.resize(num_joints_,num_joints_);
    for (uint i = 0; i < num_joints_; i++) {
        for (uint j = 0; j< num_joints_; j++) {
            if (i==j) A(i,j) = 1;
            else A(i,j) = 0;
        }
    }*/
    A.setIdentity(num_joints_,num_joints_);
    ComputeNullspace_.initialize(num_joints_, A);
    N_.resize(num_joints_,num_joints_);

    // Initialize Joint Limit Avoidance
    for (uint i = 0; i < 15; i++) ROS_INFO("JLA gain of joint %i is %f",i,JLA_gain[i]);
    JointLimitAvoidance_.initialize(ComputeJacobian_.q_min_, ComputeJacobian_.q_max_, JLA_gain, JLA_workspace);
    ROS_INFO("Joint limit avoidance initialized");

    // Initialize Posture Controller
    for (uint i = 0; i < num_joints_; i++) posture_gain[i] = 1;
    PostureControl_.initialize(ComputeJacobian_.q_min_, ComputeJacobian_.q_max_, posture_q0, posture_gain);
    ROS_INFO("Posture Control initialized");

    // Resize additional variables
    F_task_.resize(0);
    qdot_reference_.resize(num_joints_);
    q_reference_.resize(num_joints_);
    Jacobian_.resize(12,num_joints_);                // Jacobian is resized to zero rows, number of rows depends on number of tasks
    tau_.resize(num_joints_);
    tau_nullspace_.resize(num_joints_);
    isactive_vector_.resize(2);
    F_task_.resize(12);
    previous_num_active_tasks_ = 0;

    ROS_INFO("Whole Body Controller Initialized");

    return true;

}

bool WholeBodyController::addConstraint(Constraint* constraint) {
    if (!constraint->initialize(chains_, components_)) {
        return false;
    }
    constraints_.push_back(constraint);
    return true;
}

void WholeBodyController::setMeasuredJointPosition(const std::string& joint_name, double pos) {
    q_current_(joint_name_to_index_[joint_name]) = pos;
}

bool WholeBodyController::update() {

    // Set some variables to zero
    tau_.setZero();
    tau_nullspace_.setZero();   
    F_task_.setZero();

    // Update the torque output
    // Currently only one torque 'source', when multiple ones a smarter solution has to be found
    // Plan of attack: set tau to zero --> add in every update loop.
    // Note that nullspace solution should be included in various subproblems
    // Not sure if this is the right approach: summing the taus up and then doing nullspace projection may result in smoother transitions and is probably quicker

    for(std::vector<Chain*>::iterator it_chain = chains_.begin(); it_chain != chains_.end(); ++it_chain) {
        Chain* chain = *it_chain;
        chain->removeCartesianTorques();
    }

    for(std::vector<Constraint*>::iterator it_constr = constraints_.begin(); it_constr != constraints_.end(); ++it_constr) {
        Constraint* constraint = *it_constr;
        if (constraint->isActive()) {
            constraint->apply();
        }
    }

    Eigen::VectorXd torque_full(chains_.size() * 6);
    Eigen::VectorXd jacobian_full(chains_.size() * 6, num_joints_);
    jacobian_full.setZero();

    for(unsigned int i_chain = 0; i_chain < chains_.size(); ++i_chain) {
        Chain* chain = chains_[i_chain];
        chain->addToCartesianTorque(torque_full);
        chain->addToJacobian(jacobian_full);
        chain->setMeasuredJointPositions(q_current_);
    }

    tau_ = jacobian_full.transpose() * torque_full;

    //for (uint i = 0; i < tau_.rows(); i++) ROS_INFO("Task torques (%i) = %f",i,tau_(i));

    ComputeNullspace_.update(jacobian_full, N_);
    //ROS_INFO("Nullspace updated");

    JointLimitAvoidance_.update(q_current_, tau_nullspace_);
    ///ROS_INFO("Joint limit avoidance updated");

    PostureControl_.update(q_current_, tau_nullspace_);
    ///ROS_INFO("Posture control updated");

    tau_ += N_ * tau_nullspace_;
    //for (uint i = 0; i < tau_.rows(); i++) ROS_INFO("Total torques (%i) = %f",i,tau_(i));

    AdmitCont_.update(tau_, qdot_reference_, q_current_, q_reference_);

    //for (uint i = 0; i < qdot_reference_.rows(); i++) ROS_INFO("qd joint %i = %f",i,qdot_reference_(i));
    //for (uint i = 0; i < q_current_.rows(); i++) ROS_INFO("Position joint %i = %f",i,q_current_(i));
    //for (uint i = 0; i < q_reference_.rows(); i++) ROS_INFO("Joint %i = %f",i,q_reference_(i));
    publishReferences();

    return true;

}

void WholeBodyController::publishReferences() {

    for(std::vector<Component*>::iterator it_comp = components_.begin(); it_comp != components_.end(); ++it_comp) {
        Component* comp = *it_comp;
        comp->publishReferences();
    }

    /*
    // Base

    // Torso
    ///torso_msg.stop = 0;
    std::string component_name("torso");
    for (int i = 0; i<component_map_[component_name].number_of_joints; i++) {
        torso_msg_.position[i] = q_reference_(component_map_[component_name].start_index + i);
    }
    torso_pub_.publish(torso_msg_);
    ///ROS_INFO("Torso position = %f, reference = %f",component_description_map_[component_name].q[0],torso_msg.pos);

    // Left Arm
    component_name = "left_arm";
    for (int i = 0; i<component_map_[component_name].number_of_joints; i++) {
        left_arm_msg_.position[i] = q_reference_(component_map_[component_name].start_index + i);
    }
    left_arm_pub_.publish(left_arm_msg_);

    // Right Arm
    component_name = "right_arm";
    for (int i = 0; i<component_map_[component_name].number_of_joints; i++) {
        right_arm_msg_.position[i] = q_reference_(component_map_[component_name].start_index + i);
    }
    right_arm_pub_.publish(right_arm_msg_);
    //ROS_WARN("Right arm message not published");

    // Head
    */

}
