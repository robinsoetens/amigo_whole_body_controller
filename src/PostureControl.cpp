#include "PostureControl.h"
#include <ros/ros.h>

PostureControl::PostureControl() {

}

PostureControl::~PostureControl() {

}

void PostureControl::initialize(const KDL::JntArray& q_min, const KDL::JntArray& q_max, const std::vector<double>& q0, const std::vector<double>& gain, const std::map<std::string, unsigned int>& joint_name_to_index) {

    num_joints_ = q_min.rows();
    q0_.resize(num_joints_);
    q0_default_.resize(num_joints_);
    q_min_.resize(num_joints_);
    q_max_.resize(num_joints_);
    K_.resize(num_joints_);
    joint_name_to_index_ = joint_name_to_index;
    for (uint i = 0; i < num_joints_; i++) {
        q0_[i] = q0[i];
        q0_default_[i] = q0[i];
        q_min_[i] = q_min(i);
        q_max_[i] = q_max(i);
    }

    ROS_INFO("Length joint array = %i",num_joints_);
    for (uint i = 0; i < num_joints_; i++) {
        K_[i] = gain[i] / ((q_max(i) - q_min(i))*(q_max(i) - q_min(i)));
        //ROS_INFO("qmin = %f, qmax = %f, q0 = %f, K = %f, gain = %f",q_min(i),q_max(i),q0_[i], K_[i], gain[i]);
    }
    //ROS_INFO("Length joint array = %i",num_joints_);

    current_cost_ = 0;

}

void PostureControl::update(const KDL::JntArray& q_in, Eigen::VectorXd& tau_out) {

    current_cost_ = 0;
    Eigen::VectorXd tau_ps;
    tau_ps.resize(num_joints_);
    for (uint i = 0; i < num_joints_; i++) {
        tau_out(i) += K_[i]*(q0_[i] - q_in(i));
        //ROS_INFO("q(%i) = %f\tq0 = %f\ttau = %f",i,q_in(i),q0_[i],tau_out(i));

        // Add this to the costs
        current_cost_ += fabs(tau_out(i));
    }

}

bool PostureControl::setJointTarget(const std::string& joint_name, const double& value) {

    // ToDo: return bool

    ROS_INFO("Posture controller: setting joint targets");
    /// Get joint index
    std::map<std::string, unsigned int>::const_iterator index_iter = joint_name_to_index_.find(joint_name);
    if (index_iter != joint_name_to_index_.end())
    {
        unsigned int index = index_iter->second;
        /// Check whether value is between bounds
        if (value < q_min_[index] || value > q_max_[index]) {
            ROS_WARN("Joint value %f is not between bounds [%f\t%f]",value,q_min_[index],q_max_[index]);
            return false;
        }
        else {
            ROS_INFO("Posture controller: %s (%i) to %f", joint_name.c_str(), (int)index, value);
            q0_[index] = value;
        }
    } else {
        ROS_ERROR("Joint %s not in posture controller", joint_name.c_str());
        return false;
    }
    return true;

}

double PostureControl::getCost() {

    return current_cost_;
}
