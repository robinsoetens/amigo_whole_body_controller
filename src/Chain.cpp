#include "Chain.h"

Chain::Chain() : chain_jnt_to_jac_solver_(0), joint_positions_(0) {
}

Chain::~Chain() {
    delete chain_jnt_to_jac_solver_;
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
        //std::cout << i << " : " << joint_positions_(i) << std::endl;
    }
}
