#include "WholeBodyController.h"

// tf
#include <tf/transform_listener.h>

using namespace std;

const double loop_rate_ = 50;

CartesianImpedance* cart_imp_left_;
CartesianImpedance* cart_imp_right_;
CollisionAvoidance* collision_avoidance;

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

void jointMeasurementCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(unsigned int i = 0; i < msg->name.size(); ++i)
    {
        wbc->setMeasuredJointPosition(msg->name[i], msg->position[i]);
    }
}

void octoMapCallback(const octomap_msgs::OctomapBinary::ConstPtr& msg)
{
    collision_avoidance->setOctoMap(*msg);
}

void publishJointReferences(const Eigen::VectorXd& joint_refs, const vector<std::string>& joint_names)
{
    for(map<string, JointRefPublisher*>::iterator it_pub = JOINT_NAME_TO_PUB.begin(); it_pub != JOINT_NAME_TO_PUB.end(); ++it_pub)
    {
        it_pub->second->msg_ = sensor_msgs::JointState();
    }

    for(unsigned int i = 0; i < joint_refs.size(); ++i) {
        map<string, JointRefPublisher*>::iterator it_pub = JOINT_NAME_TO_PUB.find(joint_names[i]);
        it_pub->second->msg_.name.push_back(joint_names[i]);
        it_pub->second->msg_.position.push_back(joint_refs[i]);
        //cout << joint_names[i] << ": " << joint_refs[i] << endl;
    }

    for(map<string, JointRefPublisher*>::iterator it_pub = JOINT_NAME_TO_PUB.begin(); it_pub != JOINT_NAME_TO_PUB.end(); ++it_pub)
    {
        it_pub->second->pub_.publish(it_pub->second->msg_);
    }
}

void leftCancelCB()
{
    //is_active_ = false;
    server_left_->setPreempted();
}

void rightCancelCB()
{
    //is_active_ = false;
    server_right_->setPreempted();
}

void setTarget(const amigo_arm_navigation::grasp_precomputeGoal& goal, const std::string& end_effector_frame)
{

    geometry_msgs::PoseStamped goal_pose;

    goal_pose.header = goal.goal.header;
    goal_pose.pose.position.x = goal.goal.x;
    goal_pose.pose.position.y = goal.goal.y;
    goal_pose.pose.position.z = goal.goal.z;
    double roll = goal.goal.roll;
    double pitch = goal.goal.pitch;
    double yaw = goal.goal.yaw;
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    goal_pose.pose.orientation = orientation;

    if (end_effector_frame == "/grippoint_left") {
        cart_imp_left_->setGoal(goal_pose);
    }
    else if (end_effector_frame == "/grippoint_right") {
        cart_imp_right_->setGoal(goal_pose);
    }
    else ROS_WARN("Cannot process this goal");
}

void cancelTarget(const std::string& end_effector_frame) {
    if (end_effector_frame == "/grippoint_left") {
        cart_imp_left_->cancelGoal();
    }
    else if (end_effector_frame == "/grippoint_right") {
        cart_imp_right_->cancelGoal();
    }
    else ROS_WARN("Not clear what to cancel");
}

void leftGoalCB() {
    ROS_INFO("Received left goal");
    std::string end_effector_frame = "/grippoint_left";
    const amigo_arm_navigation::grasp_precomputeGoal& goal = *server_left_->acceptNewGoal();
    setTarget(goal, end_effector_frame);
}

void rightGoalCB() {
    ROS_INFO("Received right goal");
    std::string end_effector_frame = "/grippoint_right";
    const amigo_arm_navigation::grasp_precomputeGoal& goal = *server_right_->acceptNewGoal();
    setTarget(goal, end_effector_frame);
}

void loadParameterFiles(CollisionAvoidance::collisionAvoidanceParameters &ca_param)
{
    ros::NodeHandle n("~");
    std::string ns = ros::this_node::getName();
    n.param<double> (ns+"/collision_avoidance/self_collision/F_max", ca_param.self_collision.f_max, 1.0);
    n.param<double> (ns+"/collision_avoidance/self_collision/d_threshold", ca_param.self_collision.d_threshold, 1.0);
    n.param<int> (ns+"/collision_avoidance/self_collision/order", ca_param.self_collision.order, 1);

    n.param<double> (ns+"/collision_avoidance/environment_collision/F_max", ca_param.environment_collision.f_max, 1.0);
    n.param<double> (ns+"/collision_avoidance/environment_collision/d_threshold", ca_param.environment_collision.d_threshold, 1.0);
    n.param<int> (ns+"/collision_avoidance/environment_collision/order", ca_param.environment_collision.order, 1);
    n.getParam(ns+"/collision_avoidance/environment_collision/BBX/x", ca_param.environment_collision.BBX.x);
    n.getParam(ns+"/collision_avoidance/environment_collision/BBX/y", ca_param.environment_collision.BBX.y);
    n.getParam(ns+"/collision_avoidance/environment_collision/BBX/z", ca_param.environment_collision.BBX.z);
    n.getParam("/map_3d/resolution", ca_param.environment_collision.octomap_resolution);

}

int main(int argc, char **argv) {

    // Initialize node
    ros::init(argc, argv, "whole_body_controller");
    ros::NodeHandle nh_private("~");

    ros::Subscriber sub_left_arm  = nh_private.subscribe<sensor_msgs::JointState>("/arm_left_controller/measurements", 10, &jointMeasurementCallback);
    ros::Subscriber sub_right_arm = nh_private.subscribe<sensor_msgs::JointState>("/arm_right_controller/measurements", 10, &jointMeasurementCallback);
    ros::Subscriber sub_torso     = nh_private.subscribe<sensor_msgs::JointState>("/torso_controller/measurements", 10, &jointMeasurementCallback);
    ros::Subscriber sub_octomap   = nh_private.subscribe<octomap_msgs::OctomapBinary>("/octomap_binary", 10, &octoMapCallback);


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


    // Load parameter files
    CollisionAvoidance::collisionAvoidanceParameters ca_param;
    loadParameterFiles(ca_param);

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

    collision_avoidance = new CollisionAvoidance(ca_param, 1/loop_rate_);
    if (!wbc->addMotionObjective(collision_avoidance)) {
        ROS_ERROR("Could not initialize collision avoidance");
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
    delete collision_avoidance;

    delete server_left_;
    delete server_right_;
    delete wbc;

    return 0;
}
