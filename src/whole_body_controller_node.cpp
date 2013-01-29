#include "WholeBodyController.h"

// tf
#include <tf/transform_listener.h>

using namespace std;

const double loop_rate_ = 50;

CartesianImpedance* cart_imp_left_;
CartesianImpedance* cart_imp_right_;

typedef actionlib::SimpleActionServer<amigo_arm_navigation::grasp_precomputeAction> action_server;
action_server* server_left_;
action_server* server_right_;

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

void leftCancelCB() {
    //is_active_ = false;
    server_left_->setPreempted();
}

void rightCancelCB() {
    //is_active_ = false;
    server_right_->setPreempted();
}

void leftGoalCB() {
    ROS_INFO("Received left goal");
    std::string end_effector_frame = "/grippoint_left";
    const amigo_arm_navigation::grasp_precomputeGoal& goal = *server_left_->acceptNewGoal();
    wbc->setTarget(goal, end_effector_frame);
}

void rightGoalCB() {
    ROS_INFO("Received right goal");
    std::string end_effector_frame = "/grippoint_right";
    const amigo_arm_navigation::grasp_precomputeGoal& goal = *server_right_->acceptNewGoal();
    wbc->setTarget(goal, end_effector_frame);
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
    JointRefPublisher* pub_torso = new JointRefPublisher("/torso_controller/references");

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

    wbc = new WholeBodyController(1/loop_rate_);

    ros::Rate r(loop_rate_);

    server_left_ = new action_server(nh_private, "/grasp_precompute_left", false);
    server_left_->registerGoalCallback(boost::bind(&leftGoalCB));
    server_left_->registerPreemptCallback(boost::bind(&leftCancelCB));

    server_right_ = new action_server(nh_private, "/grasp_precompute_right", false);
    server_right_->registerGoalCallback(boost::bind(&rightGoalCB));
    server_right_->registerPreemptCallback(boost::bind(&rightCancelCB));

    server_left_->start();
    server_right_->start();

    tf::TransformListener tf_listener;

    cart_imp_left_ = new CartesianImpedance("grippoint_left");
    if (!wbc->addMotionObjective(cart_imp_left_)) {
        ROS_ERROR("Could not initialize cartesian impedance for left arm");
        exit(-1);
    }

    cart_imp_right_ = new CartesianImpedance("grippoint_right");
    if (!wbc->addMotionObjective(cart_imp_right_)) {
        ROS_ERROR("Could not initialize cartesian impedance for right arm");
        exit(-1);
    }

    ObstacleAvoidance* obstacle_avoidance_left = new ObstacleAvoidance("grippoint_left", &tf_listener);
    if (!wbc->addMotionObjective(obstacle_avoidance_left)) {
        ROS_ERROR("Could not initialize obstacle avoidance");
        exit(-1);
    }

    ObstacleAvoidance* obstacle_avoidance_right = new ObstacleAvoidance("grippoint_right", &tf_listener);
    if (!wbc->addMotionObjective(obstacle_avoidance_right)) {
        ROS_ERROR("Could not initialize obstacle avoidance");
        exit(-1);
    }

    KDL::JntArray q_current;
    Eigen::VectorXd q_ref;
    Eigen::VectorXd qdot_ref;

    while(ros::ok()) {

        ros::spinOnce();

        wbc->update(q_current, q_ref, qdot_ref);

        if (cart_imp_left_->status_ == 1 && server_left_->isActive()) server_left_->setSucceeded();
        if (cart_imp_right_->status_ == 1 && server_right_->isActive()) server_right_->setSucceeded();

        publishJointReferences(wbc->getJointReferences(), wbc->getJointNames());

        r.sleep();
    }

    delete cart_imp_right_;
    delete cart_imp_left_;

    delete server_left_;
    delete server_right_;
    delete wbc;

    return 0;
}
