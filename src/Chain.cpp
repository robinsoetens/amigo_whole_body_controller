#include "Chain.h"
#include "Component.h"

using namespace std;

Chain::Chain() : jnt_to_jac_solver_(0) {
    end_effector_torque_.setZero();
}

Chain::~Chain() {
    delete jnt_to_jac_solver_;
}

unsigned int Chain::getNumJoints() const {
    return joint_positions_.rows();
}

void Chain::setMeasuredJointPositions(const Eigen::VectorXd& all_joint_measurements) {
    for(unsigned int i = 0; i < joint_names_.size(); ++i) {
        joint_positions_(i) = all_joint_measurements(joint_chain_index_to_full_index_[i]);
    }
}

void Chain::addToJacobian(Eigen::VectorXd& jacobian) const {
    // Solve Chain Jacobian
    KDL::Jacobian chain_jacobian;
    jnt_to_jac_solver_->JntToJac(joint_positions_, chain_jacobian);

    unsigned int link_start_index = jacobian.rows();
    jacobian.resize(jacobian.rows() + 6, jacobian.columns());

    for(unsigned int i = 0; i < joint_names_.size(); ++i) {
        unsigned int joint_index = joint_chain_index_to_full_index_[i];
        jacobian.block(link_index_start, i, 6, i) =
                chain_jacobian.data.block(0, joint_index, 6, joint_index);
        link_start_index += 6;
    }
}

void Chain::addToCartesianTorque(Eigen::VectorXd& torque) {
    unsigned int link_start_index = torque.rows();
    torque.resize(torque.rows() + 6 * cartesian_torque_.size());

    for(unsigned int i = 0; i < cartesian_torque_.size(); ++i) {
        torque.block(link_start_index, 0, 6, 0) = cartesian_torque_[i].block(0, 0, 6, 0);
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
