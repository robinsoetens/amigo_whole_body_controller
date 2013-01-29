/*!
 * \author Janno Lunenburg
 * \date October, 2012
 * \version 0.1
 */

#ifndef POSTURE_CONTROLE_
#define POSTURE_CONTROLE_

#include <Eigen/Core>
#include <vector>
#include <kdl/jntarray.hpp>

class PostureControl {

public:
    //! Constructor
    PostureControl();

    //! Deconstructor
    virtual ~PostureControl();

    //! Initialize();
    void initialize(const KDL::JntArray& q_min, const KDL::JntArray& q_max, const std::vector<double>& q0, const std::vector<double>& gain);

    //! Update
    void update(const KDL::JntArray& q_in, Eigen::VectorXd& tau_out);

    /**
      * Returns cost, i.e., the absolute value of the torque of every single plugin
      */
    double getCost();

protected:

    //! Vector that hold the multiplication factor
        std::vector<double> K_;

        //! Joint array with average joint value
        // H = ((q-q0)/(qmax-qmin))^2
        // dH/dq = 2(q-q0)/(qmax-qmin)^2
        std::vector<double> q0_;

        //! Number of joints
        uint num_joints_;

        /**
          * Current cost
          */
        double current_cost_;

};

#endif
