#ifndef WBC_MOTIONOBJECTIVE_H_
#define WBC_MOTIONOBJECTIVE_H_

#include "Chain.h"
#include "Component.h"
#include "RobotState.h"

// Eigen
#include <Eigen/Core>

class MotionObjective {

public:

    MotionObjective();

    virtual ~MotionObjective();

    virtual bool initialize() = 0;

    virtual bool isActive() = 0;

    virtual void apply(const RobotState& robotstate) = 0;

};

#endif
