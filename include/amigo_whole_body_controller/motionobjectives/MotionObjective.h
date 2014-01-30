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

    /** Returns status
     * @return Current status of this objective */
    unsigned int getStatus();

    /** Sets status
     * @param Desired status() */
    void setStatus(unsigned int status);

    /**
     * Type of the motion objective
     */
    std::string type_;

    // ToDo: can we do this any other way? This makes no sense for collision avoidance...
    std::string tip_frame_;
    std::string root_frame_;

protected:

    /** Status for this motion objective
      * IDLE=0
      * AT_GOAL_POSE=1
      * MOVING_TO_GOAL_POSE=2 */
    unsigned int status_;

};

#endif
