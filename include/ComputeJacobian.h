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

class ComputeJacobian {
public:

    /*
     * Constructor
     */
    ComputeJacobian();

    /*
     * Deconstructor
     */
    ~ComputeJacobian();

    /*
     * Initialize
     */
    bool Initialize();

private:

    //! Array containing interfaces between various components
    std::string chain_array;

};

#endif
