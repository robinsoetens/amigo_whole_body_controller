#ifndef WBC_MOTIONOBJECTIVE_H_
#define WBC_MOTIONOBJECTIVE_H_

#include "RobotState.h"

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

    /** Returns current cost
      * This value depends on the implementation
      * Returns 0.0 if not implemented */
    virtual double getCost();

    /** Returns the torques of this motion objective */
    virtual Eigen::VectorXd getTorques();

    /** Returns relevant Jacobian matrix */
    virtual Eigen::MatrixXd getJacobian();

    /** Returns the priority of this motion objective
     * @return priority
     */
    unsigned int getPriority();

    /**
     * Type of the motion objective
     */
    std::string type_;

    // ToDo: can we do this any other way? This makes no sense for collision avoidance...
    std::string tip_frame_;
    std::string root_frame_;

    /** Priority (main idea from Dietrich 2012, although not yet fully implemented)
     * 1: Safety (i.e., collision avoidance)
     * 2: Physical constraints (i.e., joint limits)
     * 3: Task execution (i.e., Cartesian Impedance, possibly joint goals)
     * 4: Posture
     */
    unsigned int priority_;

protected:

    /** Status for this motion objective
      * IDLE=0
      * AT_GOAL_POSE=1
      * MOVING_TO_GOAL_POSE=2 */
    unsigned int status_;

    /** Vector containing all torques */
    Eigen::VectorXd torques_;//ToDo: initialize in constructor MotionObjectives?

    /** Jacobian matrix relevant for this motion objective */
    Eigen::MatrixXd jacobian_;//ToDo: initialize in constructor MotionObjectives?

    /** Costs associated with this motion objective */
    double cost_;

};

#endif
