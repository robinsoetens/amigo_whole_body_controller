#include "PostureControl.h"
#include <ros/ros.h>

PostureControl::PostureControl() {

}

PostureControl::~PostureControl() {

}

void PostureControl::initialize(const std::vector<double>& q_min, const std::vector<double>& q_max, const std::vector<double>& gain) {

    num_joints_ = q_min.size();
    q0_.resize(num_joints_);
    K_.resize(num_joints_);

    // ToDo: make this variable
    q0_(0) = -0.1;
    q0_(1) = -0.2;
    q0_(2) = 0.2;
    q0_(3) = 0.8;
    q0_(4) = 0.0;
    q0_(5) = 0.0;
    q0_(6) = 0.0;
    q0_(7) = 0.35;
    q0_(8) = -0.1;
    q0_(9) = -0.2;
    q0_(10) = 0.2;
    q0_(11) = 0.8;
    q0_(12) = 0.0;
    q0_(13) = 0.0;
    q0_(14) = 0.0;

    ROS_INFO("Length joint array = %i",num_joints_);
    for (uint i = 0; i < num_joints_; i++) {
        K_[i] = 2*gain[i] / ((q_max[i] - q_min[i])*(q_max[i] - q_min[i]));
        ROS_INFO("qmin = %f, qmax = %f, q0 = %f, K = %f",q_min[i],q_max[i],q0_(i), K_[i]);
    }
    //ROS_INFO("Length joint array = %i",num_joints_);


}

void PostureControl::update(const KDL::JntArray& q_in, Eigen::VectorXd& tau_out) {

    for (uint i = 0; i < num_joints_; i++) {
        tau_out(i) += K_[i]*(q0_(i) - q_in(i));
        //ROS_INFO("q(%i) = %f\tq0 = %f\ttau = %f ",i,q_in(i),q0_(i),tau_out(i));
    }

}

