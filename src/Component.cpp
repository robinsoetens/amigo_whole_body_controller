#include "Component.h"

Component::Component(const std::string& name) : name(name) {
    joint_names.resize(number_of_joints);
}

Component::~Component() {

}

void Component::setRootLinkName(const std::string& joint_name) {
    root_link_name_ = joint_name;
}

std::string Component::getName() const {
    return name;
}

void Component::addLeafLinkName(const std::string& leaf_name) {
    leaf_link_names_.push_back(leaf_name);
}

void Component::addJoint(int index, const std::string& joint) {
    if (index >= joint_names.size()) {
        joint_names.resize(index + 1);
    }
    joint_names[index] = joint;
    joint_name_map_[joint] = index;

    // prepare reference message
    reference_msg_.name.resize(joint_names.size());
    //reference_msg_.position.resize(joint_names.size());   // currently not used
    //reference_msg_.velocity.resize(joint_names.size());   // currently not used
    //reference_msg_.effort.resize(joint_names.size());    // currently not used
    for (int i = 0; i < joint_names.size(); ++i) {
        reference_msg_.name[i] = joint_name;
    }
}

const KDL::JntArray& Component::getJointArray() const {
    return q_current_;
}

void Component::setMeasurementTopic(const std::string& topic_name) {
    ros::NodeHandle n;
    n.subscribe<sensor_msgs::JointState>(topic_name, 1, &Component::measurementCallback, this);
}


void Component::setReference(const std::string& topic_name) {
    ros::NodeHandle n;
    n.advertise<sensor_msgs::JointState>(topic_name, 10);
}

void Component::measurementCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for (int i = 0; i <  number_of_joints_; ++i) {
        setJointPosition(msg->name[i], msg->position[i]);
    }
}

void Component::setJointPosition(const std::string& joint_name, double pos) {
    q_current_(joint_name_map[joint_name]) = pos;
}
