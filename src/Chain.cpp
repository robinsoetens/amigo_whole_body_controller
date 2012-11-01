#include "Chain.h"
#include "Component.h"

using namespace std;

Chain::Chain() : num_joints_(0), jnt_to_jac_solver_(0), end_effector_torque_(6) {
    end_effector_torque_.setZero();
}

Chain::~Chain() {
    delete jnt_to_jac_solver_;
}

void Chain::addComponent(Component* component) {
    components_.push_back(component);
    num_joints_ += component->getNumJoints();
}

const std::vector<Component*>& Chain::getComponents() const {
    return components_;
}

unsigned int Chain::getNumJoints() const {
    return num_joints_;
}

KDL::Jacobian Chain::getJacobian() const {
    KDL::JntArray q_chain(num_joints_);

    uint current_index = num_joints_; // TODO: get rid of current_index (see also WholeBodyController::update)
    for(vector<Component*>::const_iterator it_comp = components_.begin(); it_comp != components_.end(); ++it_comp) {
        Component* component = *it_comp;

        const KDL::JntArray& q_component = component->getJointArray();

        current_index -= component->getNumJoints();
        for(unsigned int i = 0; i< component->getNumJoints(); i++) {
            q_chain(current_index + i) = q_component(i);
        }
    }

    // Solve Chain Jacobian
    KDL::Jacobian chain_jacobian;
    jnt_to_jac_solver_->JntToJac(q_chain, chain_jacobian);

    return chain_jacobian;
}

void Chain::addEndEffectorTorque(const Eigen::VectorXd& torque) {
    end_effector_torque_ += torque;
}

const Eigen::VectorXd& Chain::getEndEffectorTorque() {
    return end_effector_torque_;
}

void Chain::removeEndEffectorTorque() {
    end_effector_torque_.setZero();
}

Component* Chain::getRootComponent() const {
    return components_.back();
}

Component* Chain::getLeafComponent() const {
    return components_.front();
}
