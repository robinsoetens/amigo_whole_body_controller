#include "JointLimitAvoidance.h"
#include <ros/ros.h>

JointLimitAvoidance::JointLimitAvoidance() {

}

JointLimitAvoidance::~JointLimitAvoidance() {

}

/*void JointLimitAvoidance::initialize(const KDL::JntArray& q_min, const KDL::JntArray& q_max, const std::vector<double>& gain) {

    num_joints_ = q_min.data.rows();
    q0_.resize(num_joints_);
    K_.resize(num_joints_);

    ROS_INFO("Length joint array = %i",num_joints_);
    for (uint i = 0; i < num_joints_; i++) {
        q0_(i) = (q_min(i)+q_max(i))/2;
        ROS_INFO("qmin = %f, qmax = %f, q0 = %f",q_min(i),q_max(i),q0_(i));
        K_[i] = 2*gain[i] / (q_max(i) - q_min(i))*(q_max(i) - q_min(i));
    }
    ROS_INFO("Joint limit avoidance initialized");

}*/
void JointLimitAvoidance::initialize(const std::vector<double>& q_min, const std::vector<double>& q_max, const std::vector<double>& gain) {

    num_joints_ = q_min.size();
    q0_.resize(num_joints_);
    K_.resize(num_joints_);

    ROS_INFO("Length joint array = %i",num_joints_);
    for (uint i = 0; i < num_joints_; i++) {
        q0_(i) = (q_min[i]+q_max[i])/2;
        K_[i] = 2*gain[i] / ((q_max[i] - q_min[i])*(q_max[i] - q_min[i]));
        ROS_INFO("qmin = %f, qmax = %f, q0 = %f, K = %f",q_min[i],q_max[i],q0_(i), K_[i]);
    }
    //ROS_INFO("Length joint array = %i",num_joints_);
    ROS_INFO("Joint limit avoidance initialized");

}

void JointLimitAvoidance::update(const KDL::JntArray& q_in, Eigen::VectorXd& tau_out) {

    //ToDo: Check joint limit avoidance algorithm
    for (uint i = 0; i < num_joints_; i++) {
        tau_out(i) = K_[i]*(q0_(i) - q_in(i));
        //ROS_INFO("q(%i) = %f\tq0 = %f\ttau = %f ",i,q_in(i),q0_(i),tau_out(i));
    }

}
