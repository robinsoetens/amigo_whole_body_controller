#include "Component.h"

Component::Component(const std::string& name) : name(name) {
}

Component::~Component() {
}

std::string Component::getName() const {
    return name;
}

void Component::setRootLinkName(const std::string& joint_name) {
    root_link_name_ = joint_name;
}

std::string Component::getRootLinkName() const {
    return root_link_name_;
}

void Component::addLeafLinkName(const std::string& leaf_name) {
    leaf_link_names_.push_back(leaf_name);
}

const std::vector<std::string>& Component::getLeafLinkNames() {
    return leaf_link_names_;
}

void Component::addJoint(uint index, const std::string& joint_name) {
    if (index >= joint_names_.size()) {
        joint_names_.resize(index + 1);
    }
    joint_names_[index] = joint_name;
    joint_name_map_[joint_name] = index;

    // prepare reference message
    reference_msg_.name.resize(joint_names_.size());
    //reference_msg_.position.resize(joint_names.size());   // currently not used
    //reference_msg_.velocity.resize(joint_names.size());   // currently not used
    //reference_msg_.effort.resize(joint_names.size());    // currently not used
    for (unsigned int i = 0; i < joint_names_.size(); ++i) {
        reference_msg_.name[i] = joint_name;
    }
}

unsigned int Component::getNumJoints() const {
    return joint_names_.size();
}

const KDL::JntArray& Component::getJointArray() const {
    return q_current_;
}

void Component::setMeasurementTopic(const std::string& topic_name) {
    ros::NodeHandle n;
    sub_measurements_ = n.subscribe<sensor_msgs::JointState>(topic_name, 1, &Component::measurementCallback, this);
}

void Component::setReferenceTopic(const std::string& topic_name) {
    ros::NodeHandle n;
    pub_references_ = n.advertise<sensor_msgs::JointState>(topic_name, 10);
}

void Component::measurementCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for (uint i = 0; i < msg->name.size(); ++i) {
        setJointPosition(msg->name[i], msg->position[i]);
    }
}

void Component::setJointPosition(const std::string& joint_name, double pos) {
    q_current_(joint_name_map_[joint_name]) = pos;
}

void Component::publishReferences() {
    pub_references_.publish(reference_msg_);
}
