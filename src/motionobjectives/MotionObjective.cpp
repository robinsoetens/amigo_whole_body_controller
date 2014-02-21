#include "amigo_whole_body_controller/motionobjectives/MotionObjective.h"

MotionObjective::MotionObjective() {

}

MotionObjective::~MotionObjective() {

}

unsigned int MotionObjective::getStatus() {
    return status_;
}

void MotionObjective::setStatus(unsigned int status) {
    status_ = status;
}

double MotionObjective::getCost() {
    return cost_;
}

Eigen::VectorXd MotionObjective::getTorques() {
    return torques_;
}

Eigen::MatrixXd MotionObjective::getJacobian() {
    return jacobian_;
}

unsigned int MotionObjective::getPriority() {
    return priority_;
}
