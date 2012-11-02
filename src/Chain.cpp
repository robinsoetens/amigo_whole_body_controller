#include "Chain.h"
#include "Component.h"

using namespace std;

Chain::Chain() : joint_positions_(0), jnt_to_jac_solver_(0) {
}

Chain::~Chain() {
    delete jnt_to_jac_solver_;
}

void Chain::addJoint(const std::string& joint_name, const std::string& link_name, unsigned int full_joint_index) {
    joint_positions_.resize(joint_positions_.rows() + 1);
    joint_names_.push_back(joint_name);
    link_names_.push_back(link_name);
    joint_chain_index_to_full_index_.push_back(full_joint_index);
}

bool Chain::hasLink(const std::string& link_name) const {
    for(std::vector<std::string>::const_iterator it_link = link_names_.begin(); it_link != link_names_.end(); ++it_link) {
        if (*it_link == link_name) {
            return true;
        }
    }
    return false;
}

unsigned int Chain::getNumJoints() const {
    return joint_positions_.rows();
}

void Chain::setMeasuredJointPositions(const KDL::JntArray &all_joint_measurements) {
    for(unsigned int i = 0; i < joint_names_.size(); ++i) {
        joint_positions_(i) = all_joint_measurements(joint_chain_index_to_full_index_[i]);
    }
}

void Chain::addToJacobian(Eigen::MatrixXd& jacobian) const {
    // Solve Chain Jacobian
    KDL::Jacobian chain_jacobian;
    jnt_to_jac_solver_->JntToJac(joint_positions_, chain_jacobian);

    unsigned int link_start_index = jacobian.rows();
    jacobian.resize(jacobian.rows() + 6, jacobian.cols());

    // TODO: deal with multiple cartesian torques

    for(unsigned int i = 0; i < joint_names_.size(); ++i) {
        unsigned int joint_index = joint_chain_index_to_full_index_[i];
        jacobian.block(link_start_index, i, 6, i) =
                chain_jacobian.data.block(0, joint_index, 6, joint_index);
    }
    // link_start_index += 6;
}

void Chain::addToCartesianTorque(Eigen::VectorXd& torque) {
    unsigned int link_start_index = torque.rows();
    torque.resize(torque.rows() + 6 * cartesian_torques_.size());

    for(std::map<std::string, Eigen::VectorXd>::iterator it_torque = cartesian_torques_.begin(); it_torque != cartesian_torques_.end(); ++it_torque) {
        torque.block(link_start_index, 0, 6, 0) = it_torque->second.block(0, 0, 6, 0);
        link_start_index += 6;
    }
}

void Chain::addCartesianTorque(const std::string& link_name, const Eigen::VectorXd& torque) {
    std::map<std::string, Eigen::VectorXd>::iterator it_torque = cartesian_torques_.find(link_name);
    if (it_torque == cartesian_torques_.end()) {
        cartesian_torques_[link_name] = torque;
    } else {
        it_torque->second += torque;
    }
}

void Chain::removeCartesianTorques() {
    cartesian_torques_.clear();
}
