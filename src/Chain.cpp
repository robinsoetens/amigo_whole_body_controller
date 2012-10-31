#include "Chain.h"

Chain::Chain() : jnt_to_jac_solver_(0) {

}

Chain::~Chain() {
    delete jnt_to_jac_solver_;
}

void Chain::addComponent(const std::string& component_name) {
    components_.push_back(new Component(component_name));
}
