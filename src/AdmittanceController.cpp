#include "AdmittanceController.h"

#define PI 3.14159265358979
#define eps 1e-16

AdmittanceController::AdmittanceController() {

}

AdmittanceController::~AdmittanceController() {

}

void AdmittanceController::initialize(const std::vector<double>& q_min, const std::vector<double>& q_max) {

    // Resize relevant parameters
    uint num_joints = 15;
    Ts = 0.1;
    // ToDo: Make variables (use M and D as inputs?)
    m_.resize(num_joints);
    d_.resize(num_joints);
    k_.resize(num_joints);
    om_.resize(num_joints);
    a_.resize(2,num_joints);
    b_.resize(2,num_joints);
    tau_previous_.resize(num_joints);
    qdot_reference_previous_.resize(num_joints);
    q_min_.resize(num_joints);
    q_max_.resize(num_joints);

    // Set parameters (hardcoded)
    for (uint i = 0; i<num_joints; i++) {
        m_(i) = 0.1;
        d_(i) = 1;
    }
    m_(7) = 1; // Torso
    d_(7) = 10;   // Torso

    for (uint i = 0; i<num_joints; i++) {

        k_(i) = 1/d_(i);
        om_(i) = d_(i)/m_(i);

        double wp = om_(i) + eps;
        double alpha = wp/(tan(wp*Ts/2));

        double x1 = alpha/om_(i)+1;
        double x2 = -alpha/om_(i)+1;

        // Numerator and denominator of the filter
        a_(0,i) = 1;
        a_(1,i) = x2 / x1;
        b_(0,i) = 1  / x1;
        b_(1,i) = 1  / x1;

        ///ROS_INFO("a %i = %f, %f, b %i = %f, %f", i, a_(0,i), a_(1,i), i, b_(0,1), b_(1,i));

        // Set previous in- and outputs to zero
        tau_previous_(i) = 0;
        qdot_reference_previous_(i) = 0;

        // Set joint limits
        q_min_(i) = q_min[i];
        q_max_(i) = q_max[i];

    }

    // Set inputs, states, outputs to zero
    ROS_INFO("Admittance controller initialized");

}

void AdmittanceController::update(const Eigen::VectorXd tau, Eigen::VectorXd& qdot_reference, const KDL::JntArray& q_current, Eigen::VectorXd& q_reference) {

    // Update filters --> as a result desired velocities are known
    for (int i = 0; i<tau.size(); i++) {

        qdot_reference(i)  = b_(0,i) * tau(i);
        qdot_reference(i) += b_(1,i) * tau_previous_(i);
        qdot_reference(i) -= a_(1,i) * qdot_reference_previous_(i);
        qdot_reference(i) *= k_(i);
        q_reference(i) = std::min(q_max_(i),std::max(q_min_(i),q_current(i)+Ts*qdot_reference(i)));

    }
    tau_previous_ = tau;
    qdot_reference_previous_ = qdot_reference;

    ///ROS_INFO("qdr = %f %f %f %f", qdot_reference(0), qdot_reference(1), qdot_reference(2), qdot_reference(3));
    ///ROS_INFO("qdrspindle = %f", qdot_reference(7));

}

