#include "PostureControl.h"
#include <ros/ros.h>

PostureControl::PostureControl() {

}

PostureControl::~PostureControl() {

}

void PostureControl::initialize(const KDL::JntArray& q_min, const KDL::JntArray& q_max, const std::vector<double>& q0, const std::vector<double>& gain) {

    num_joints_ = q_min.rows();
    q0_.resize(num_joints_);
    K_.resize(num_joints_);
    for (uint i = 0; i < num_joints_; i++) q0_[i] = q0[i];

    ROS_INFO("Length joint array = %i",num_joints_);
    for (uint i = 0; i < num_joints_; i++) {
        K_[i] = 2*gain[i] / ((q_max(i) - q_min(i))*(q_max(i) - q_min(i)));
        //ROS_INFO("qmin = %f, qmax = %f, q0 = %f, K = %f",q_min(i),q_max(i),q0_[i], K_[i]);
    }
    //ROS_INFO("Length joint array = %i",num_joints_);

    current_cost_ = 0;

}

void PostureControl::update(const KDL::JntArray& q_in, Eigen::VectorXd& tau_out) {

    current_cost_ = 0;

    for (uint i = 0; i < num_joints_; i++) {
        tau_out(i) += 100*K_[i]*(q0_[i] - q_in(i));                             // added a gain of 100
        //ROS_INFO("q(%i) = %f\tq0 = %f\ttau = %f ",i,q_in(i),q0_[i],tau_out(i));

        // Add this to the costs
        current_cost_ += fabs(tau_out(i));
    }

}

double PostureControl::getCost() {

    return current_cost_;
}
