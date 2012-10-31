#ifndef WBC_COMPONENT_H_
#define WBC_COMPONENT_H_

#include <ros/ros.h>

#include <string>

class Component {

public:

    Component(const std::string& name);

    virtual ~Component();

    void setRootLinkName(const std::string& joint_name);

    std::string getName() const;

    void addLeafLinkName(const std::string& leaf_name);

    void addJoint(int index, const std::string& joint);

    const KDL::JntArray& getJointArray() const;

    void setMeasurementTopic(const std::string& topic_name);

    void setReferenceTopic(const std::string& topic_name);

    void measurementCallback(const sensor_msgs::JointState::ConstPtr& msg);

    int start_index;

protected:

    std::string name;

    std::string root_link_name_;

    std::vector<std::string> leaf_link_names_;

    std::vector<std::string> joint_names;

    std::map<std::string, int> joint_name_map_;

    KDL::JntArray q_current_;

    ros::Publisher pub_references_;

    ros::Subscriber sub_measurements_;

    sensor_msgs::JointState reference_msg_;

    void setJointPosition(const std::string& joint_name, double pos);

};

#endif
