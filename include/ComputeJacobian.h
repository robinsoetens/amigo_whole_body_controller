/*!
 * \author Janno Lunenburg
 * \date September, 2012
 * \version 0.1
 */

#ifndef COMPUTE_JACOBIAN_H_
#define COMPUTE_JACOBIAN_H_

// ROS
#include <ros/ros.h>

// Construct Chain
#include "ConstructChain.h"

#include "TreeDescription.h"

// KDL
#include <kdl/chainjnttojacsolver.hpp>

// Vector and map
#include <vector>
#include <map>

#include "Chain.h"
#include "Component.h"

class ComputeJacobian {
public:

    /*
     * Constructor
     */
    ComputeJacobian();

    /*
     * Deconstructor
     */
   virtual ~ComputeJacobian();

    /*
     * Initialize
     */
    bool Initialize(std::map<std::string, Component*>& component_map, std::map<std::string, uint>& joint_name_index_map);

    //void Update(std::map<std::string, std::vector<double> >);
    // Why can't I make the update const &?
    void Update(const KDL::JntArray& q_current, const std::map<std::string, Component*> component_map, Eigen::MatrixXd& Jacobian);

    //! Map contains a string to describe which component this concerns and a vector with eventually two integers to describe the start and end-index of this component
    std::map<std::string, std::vector<int> > index_map;

    //!Number of joints
    unsigned int num_joints;//, num_manipulator_joints, num_torso_joints;

    //! Joint limits
    ///KDL::JntArray joint_min, joint_max;
    std::vector<double> q_min_, q_max_;


private:

    std::vector<Chain> chains_;

};

#endif
