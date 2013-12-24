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
