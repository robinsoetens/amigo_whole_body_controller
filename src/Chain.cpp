#include "Chain.h"
#include "Component.h"

using namespace std;

Chain::Chain() : jnt_to_jac_solver_(0), joint_positions_(0) {
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
        cout << *it_link << endl;
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
    if (cartesian_torques_.empty()) {
        return;
    }

    // Solve Chain Jacobian

    //ROS_INFO("1");

    KDL::Jacobian chain_jacobian;
    chain_jacobian.resize(kdl_chain_.getNrOfJoints());
    jnt_to_jac_solver_->JntToJac(joint_positions_, chain_jacobian);

    //cout << "joint_positions_ = " << endl;
    //cout << joint_positions_.data << endl;

    //cout << "chain_jacobian = " << endl;
    //cout << chain_jacobian.data << endl;

    //ROS_INFO("joint_positions_.rows() = %d", joint_positions_.rows());

    //ROS_INFO("kdl_chain.getNrOfJoints() = %d", kdl_chain_.getNrOfJoints());

    //ROS_INFO("Chain Jacobian: rows = %d, cols = %d", chain_jacobian.rows(), chain_jacobian.columns());

    //ROS_INFO("2");

    unsigned int link_start_index = jacobian.rows();

    // The following could be much more efficient:

    Eigen::MatrixXd jacobian_new(jacobian.rows() + 6, jacobian.cols());
    jacobian_new.setZero();

    for(unsigned int i = 0; i < jacobian.rows(); ++i) {
        for(unsigned int j = 0; j < jacobian.cols(); ++j) {
            jacobian_new(i, j) = jacobian(i, j);
        }
    }
    jacobian = jacobian_new;

    //ROS_INFO("Full Jacobian: rows = %d, cols = %d", jacobian.rows(), jacobian.cols());

    //ROS_INFO("3");

    // TODO: deal with multiple cartesian torques

    for(unsigned int i = 0; i < joint_names_.size(); ++i) {
        unsigned int joint_index = joint_chain_index_to_full_index_[i];

        ROS_INFO("Chain joint index = %d, total joint index = %d", i, joint_index);

        for(unsigned int j = 0; j < 6; ++j) {
            jacobian(link_start_index + j, joint_index) = chain_jacobian(j, i);
        }

        //jacobian.block(link_start_index, joint_index, 6, joint_index) =
        //        chain_jacobian.data.block(0, i, 6, i);
    }

    //cout << "Jacobian = " << endl;
    //cout << jacobian << endl;

    // link_start_index += 6;
}

void Chain::addToCartesianTorque(Eigen::VectorXd& torque) {
    unsigned int link_start_index = torque.rows();

    Eigen::VectorXd torque_new(torque.rows() + 6 * cartesian_torques_.size());
    torque_new.setZero();

    for(unsigned int i = 0; i < torque.rows(); ++i) {
        torque_new(i) = torque(i);
    }
    torque = torque_new;

    for(std::map<std::string, Eigen::VectorXd>::iterator it_torque = cartesian_torques_.begin(); it_torque != cartesian_torques_.end(); ++it_torque) {
        //cout << "torque part = " << endl << it_torque->second << endl;
        for(unsigned int i = 0; i < 6; ++i) {
            torque(i + link_start_index) = it_torque->second(i);
        }
        link_start_index += 6;
    }

    //cout << "torque = " << endl;
    //cout << torque << endl;
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
