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
    for(unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); i++){
        if (kdl_chain_.getSegment(i).getName().data() == link_name) {
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

void Chain::fillJacobian(Eigen::MatrixXd& jacobian) const {
    if (cartesian_wrenches_.empty()) {
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

    // Add 6 new rows to the current whole body jacobian (TODO: could be much more efficient)

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

    // TODO: deal with multiple cartesian wrenches
    for(unsigned int i = 0; i < joint_names_.size(); ++i) {
        unsigned int joint_index = joint_chain_index_to_full_index_[i];

        //ROS_INFO("Chain joint index = %d, total joint index = %d", i, joint_index);

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

void Chain::fillCartesianWrench(Eigen::VectorXd& all_wrenches) {
    unsigned int link_start_index = all_wrenches.rows();

    // Add 6 new values to current whole body wrench vector (TODO: could be much more efficient)

    Eigen::VectorXd wrench_new(all_wrenches.rows() + 6 * cartesian_wrenches_.size());
    wrench_new.setZero();

    for(unsigned int i = 0; i < all_wrenches.rows(); ++i) {
        wrench_new(i) = all_wrenches(i);
    }
    all_wrenches = wrench_new;

    // Fill all_wrenches with the wrenches added to this chain

    for(std::map<std::string, Eigen::VectorXd>::iterator it_wrench = cartesian_wrenches_.begin(); it_wrench != cartesian_wrenches_.end(); ++it_wrench) {
        //cout << "torque part = " << endl << it_torque->second << endl;
        for(unsigned int i = 0; i < 6; ++i) {
            all_wrenches(i + link_start_index) = it_wrench->second(i);
        }
        link_start_index += 6;
    }

    //cout << "torque = " << endl;
    //cout << torque << endl;
}

void Chain::addCartesianWrench(const std::string& link_name, const Eigen::VectorXd& wrench) {
    std::map<std::string, Eigen::VectorXd>::iterator it_wrench = cartesian_wrenches_.find(link_name);
    if (it_wrench == cartesian_wrenches_.end()) {
        cartesian_wrenches_[link_name] = wrench;
    } else {
        it_wrench->second += wrench;
    }
}

void Chain::removeCartesianWrenches() {
    cartesian_wrenches_.clear();
}
