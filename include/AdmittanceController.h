/*!
 * \author Janno Lunenburg
 * \date September, 2012
 * \version 0.1
 */

#ifndef ADMITTANCECONTROLLER_H_
#define ADMITTANCECONTROLLER_H_

// ROS
#include "ros/ros.h"

// Eigen
#include <Eigen/Core>

// KDL
#include <kdl/chainjnttojacsolver.hpp>

class AdmittanceController {

public:

    /**
     * Constructor
     */
    AdmittanceController();

    /**
     * Deconstructor
     */
    ~AdmittanceController();

    /**
     * Initialize
     */
    void initialize();

    /**
     * Update
     */
    void update(const Eigen::VectorXd, Eigen::VectorXd&);

private:

    //! Vector containing simulated masses
    Eigen::VectorXd m_;

    //! Vector containing simulated damping
    Eigen::VectorXd d_;

    //! Vector containing resulting gains
    Eigen::VectorXd k_;

    //! Vector containing resulting pole frequency (rad/s)
    Eigen::VectorXd om_;

    //! Matrices containing numerator and denominator parameters for low pass filter
    Eigen::MatrixXd a_;
    Eigen::MatrixXd b_;

    //! Vectors containing previous in- and outputs
    Eigen::VectorXd tau_previous_;
    Eigen::VectorXd qdot_reference_previous_;

    //! Sampling time
    double Ts;

};

#endif
