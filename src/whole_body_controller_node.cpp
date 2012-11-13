#include "WholeBodyController.h"

// tf
#include <tf/transform_listener.h>

using namespace std;

const double loop_rate_ = 50;

Constraint* cart_imp_left;
Constraint* cart_imp_right;

WholeBodyController* wbc;

struct JointRefPublisher {

    JointRefPublisher(const string& ref_topic) {
        ros::NodeHandle nh;
        pub_ = nh.advertise<sensor_msgs::JointState>(ref_topic, 10);
    }

    ros::Publisher pub_;

    sensor_msgs::JointState msg_;

};

map<string, JointRefPublisher*> JOINT_NAME_TO_PUB;

void jointMeasurementCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for(unsigned int i = 0; i < msg->name.size(); ++i) {
        wbc->setMeasuredJointPosition(msg->name[i], msg->position[i]);
    }
}

void publishJointReferences(const Eigen::VectorXd& joint_refs, const vector<std::string>& joint_names) {
    for(map<string, JointRefPublisher*>::iterator it_pub = JOINT_NAME_TO_PUB.begin(); it_pub != JOINT_NAME_TO_PUB.end(); ++it_pub) {
        it_pub->second->msg_ = sensor_msgs::JointState();
    }

    for(unsigned int i = 0; i < joint_refs.size(); ++i) {
        map<string, JointRefPublisher*>::iterator it_pub = JOINT_NAME_TO_PUB.find(joint_names[i]);
        it_pub->second->msg_.name.push_back(joint_names[i]);
        it_pub->second->msg_.position.push_back(joint_refs[i]);
        //cout << joint_names[i] << ": " << joint_refs[i] << endl;
    }

    for(map<string, JointRefPublisher*>::iterator it_pub = JOINT_NAME_TO_PUB.begin(); it_pub != JOINT_NAME_TO_PUB.end(); ++it_pub) {
        it_pub->second->pub_.publish(it_pub->second->msg_);
    }
}

int main(int argc, char **argv) {

    // Initialize node
    ros::init(argc, argv, "whole_body_controller");
    ros::NodeHandle nh_private("~");

    ros::Subscriber sub_left_arm  = nh_private.subscribe<sensor_msgs::JointState>("/arm_left_controller/measurements", 10, &jointMeasurementCallback);
    ros::Subscriber sub_right_arm = nh_private.subscribe<sensor_msgs::JointState>("/arm_right_controller/measurements", 10, &jointMeasurementCallback);
    ros::Subscriber sub_torso     = nh_private.subscribe<sensor_msgs::JointState>("/torso_controller/measurements", 10, &jointMeasurementCallback);

    JointRefPublisher* pub_left_arm = new JointRefPublisher("/arm_left_controller/references");
    JointRefPublisher* pub_right_arm = new JointRefPublisher("/arm_right_controller/references");
    JointRefPublisher* pub_torso= new JointRefPublisher("/torso_controller/references");

    JOINT_NAME_TO_PUB["wrist_yaw_joint_left"] = pub_left_arm;
    JOINT_NAME_TO_PUB["wrist_pitch_joint_left"] = pub_left_arm;
    JOINT_NAME_TO_PUB["elbow_roll_joint_left"] = pub_left_arm;
    JOINT_NAME_TO_PUB["elbow_pitch_joint_left"] = pub_left_arm;
    JOINT_NAME_TO_PUB["shoulder_roll_joint_left"] = pub_left_arm;
    JOINT_NAME_TO_PUB["shoulder_pitch_joint_left"] = pub_left_arm;
    JOINT_NAME_TO_PUB["shoulder_yaw_joint_left"] = pub_left_arm;
    JOINT_NAME_TO_PUB["spindle_joint"] = pub_torso;
    JOINT_NAME_TO_PUB["wrist_yaw_joint_right"] = pub_right_arm;
    JOINT_NAME_TO_PUB["wrist_pitch_joint_right"] = pub_right_arm;
    JOINT_NAME_TO_PUB["elbow_roll_joint_right"] = pub_right_arm;
    JOINT_NAME_TO_PUB["elbow_pitch_joint_right"] = pub_right_arm;
    JOINT_NAME_TO_PUB["shoulder_roll_joint_right"] = pub_right_arm;
    JOINT_NAME_TO_PUB["shoulder_pitch_joint_right"] = pub_right_arm;
    JOINT_NAME_TO_PUB["shoulder_yaw_joint_right"] = pub_right_arm;

    wbc = new WholeBodyController();

    ros::Rate r(loop_rate_);

    tf::TransformListener tf_listener;

    CartesianImpedance* cart_imp_left = new CartesianImpedance("hand_left", &tf_listener);
    if (!wbc->addConstraint(cart_imp_left)) {
        ROS_ERROR("Could not initialize cartesian impedance for left arm");
        exit(-1);
    }

    CartesianImpedance* cart_imp_right = new CartesianImpedance("hand_right", &tf_listener);
    if (!wbc->addConstraint(cart_imp_right)) {
        ROS_ERROR("Could not initialize cartesian impedance for right arm");
        exit(-1);
    }

    tf::Stamped<tf::Pose> left_goal;
    left_goal.setOrigin(tf::Vector3(0.5, 0, 0.7));
    left_goal.setRotation(tf::Quaternion(0, 0, 0, 1));
    left_goal.frame_id_ = "/base_link";
    cart_imp_left->setGoal(left_goal);
    cart_imp_right->setGoal(left_goal);

    while(ros::ok()) {

        ros::spinOnce();

        wbc->update();

        publishJointReferences(wbc->getJointReferences(), wbc->getJointNames());

        r.sleep();
    }

    delete wbc;

    return 0;
}
