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
#include <map>

class PostureControl {

public:
    //! Constructor
    PostureControl();

    //! Deconstructor
    virtual ~PostureControl();

    //! Initialize();
    void initialize(const KDL::JntArray& q_min, const KDL::JntArray& q_max, const std::vector<double>& q0, const std::vector<double>& gain, const std::map<std::string, unsigned int>& joint_name_to_index);

    //! Update
    void update(const KDL::JntArray& q_in, Eigen::VectorXd& tau_out);

    /**
      * Sets the desired value of a joint to a specific value
      * @param joint_name Name the joint which position should be altered
      * @param value Value to which this joint should move to
      */
    void setJointTarget(const std::string &joint_name, const double &value);

    /**
      * Returns cost, i.e., the absolute value of the torque of every single plugin
      */
    double getCost();

protected:

    /**
      * Maps joint names to indexes
      */
    std::map<std::string, unsigned int> joint_name_to_index_;

    //! Vector that hold the multiplication factor
    std::vector<double> K_;

    /** Joint array with desired joint value.
      * Default value can be used in case a reset is desired
      */
    // H = ((q-q0)/(qmax-qmin))^2
    // dH/dq = 2(q-q0)/(qmax-qmin)^2
    std::vector<double> q0_, q0_default_;

    /**
      * Vectors containing lower and upper limits of joint values
      */
    std::vector<double> q_min_, q_max_;

    //! Number of joints
    uint num_joints_;

    /**
      * Current cost
      */
    double current_cost_;

};

#endif
