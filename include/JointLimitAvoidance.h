/*!
 * \author Janno Lunenburg
 * \date October, 2012
 * \version 0.1
 */

#ifndef JOINT_LIMIT_AVOIDANCE_
#define JOINT_LIMIT_AVOIDANCE_

#include <Eigen/Core>
#include <vector>
#include <kdl/jntarray.hpp>

class JointLimitAvoidance {

public:
    //! Constructor
    JointLimitAvoidance();

    //! Deconstructor
    virtual ~JointLimitAvoidance();

    //! Initialize();
    void initialize(KDL::JntArray q_min, KDL::JntArray q_max, std::vector<double> gain);

    //! Update
    void update(const KDL::JntArray& q_in, Eigen::VectorXd& tau_out);

protected:

    //! Vector that hold the multiplication factor
        std::vector<double> K_;

        //! Joint array with average joint value
        // H = ((q-q0)/(qmax-qmin))^2
        // dH/dq = 2(q-q0)/(qmax-qmin)^2
        KDL::JntArray q0_;

        //! Number of joints
        uint num_joints_;

};

#endif
