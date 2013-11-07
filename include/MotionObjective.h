#ifndef WBC_MOTIONOBJECTIVE_H_
#define WBC_MOTIONOBJECTIVE_H_

//#include "Chain.h"
//#include "Component.h"
#include "RobotState.h"

// Eigen
//#include <Eigen/Core>

class MotionObjective {

public:

    MotionObjective();

    virtual ~MotionObjective();

    virtual bool initialize(RobotState& robotstate) = 0;

    virtual void apply(RobotState& robotstate) = 0;

    /**
     * Type of the motion objective
     */
    std::string type_;

    // ToDo: can we do this any other way?
    std::string tip_frame_;
    std::string root_frame_;

};

#endif
