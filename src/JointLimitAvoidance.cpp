#include "JointLimitAvoidance.h"
#include <ros/ros.h>

JointLimitAvoidance::JointLimitAvoidance() {

}

JointLimitAvoidance::~JointLimitAvoidance() {

}

void JointLimitAvoidance::initialize(KDL::JntArray q_min, KDL::JntArray q_max, std::vector<double> gain) {

    num_joints_ = q_min.rows();
    q0_.resize(num_joints_);
    K_.resize(num_joints_);

    ///ROS_INFO("Length joint array = %i",q_min.rows());
    for (uint i = 0; i < num_joints_; i++) {
        q0_(i) = (q_min(i)+q_max(i))/2;
        K_[i] = 2*gain[i] / (q_max(i) - q_min(i))*(q_max(i) - q_min(i));
    }
    ROS_INFO("Joint limit avoidance initialized");

}

void JointLimitAvoidance::update(const KDL::JntArray& q_in, Eigen::VectorXd& tau_out) {

    //ToDo: Check joint limit avoidance algorithm
    for (uint i = 0; i < num_joints_; i++) {
        tau_out(i) = K_[i]*(q_in(i) - q0_(i));
    }
    ///ROS_INFO("Spindle: q = %f, q0 = %f, qdot_out = %f", q_in(0), q0_(0), qdot_out(0));

}
