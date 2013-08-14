#include "WholeBodyController.h"
#include <amigo_whole_body_controller/ArmTaskAction.h>

// tf
#include <tf/transform_listener.h>

using namespace std;

const double loop_rate_ = 50;

typedef actionlib::SimpleActionServer<amigo_whole_body_controller::ArmTaskAction> action_server;
action_server* add_motion_objective_server_;
CollisionAvoidance* collision_avoidance;

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

void octoMapCallback(const octomap_msgs::OctomapBinary::ConstPtr& msg)
{
    collision_avoidance->setOctoMap(*msg);
}

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

void CancelCB() {
    ROS_INFO("Canceling goal");
    add_motion_objective_server_->setPreempted();
    // ToDo: remove motion objective
}

void setTarget(const amigo_arm_navigation::grasp_precomputeGoal& goal, const std::string& end_effector_frame) {
    /*
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
    else ROS_WARN("Cannot process this goal");*/
}

void cancelTarget(const std::string& tip_frame, const std::string& root_frame) {

    /*
    if (end_effector_frame == "/grippoint_left") {
        cart_imp_left_->cancelGoal();
    }
    else if (end_effector_frame == "/grippoint_right") {
        cart_imp_right_->cancelGoal();
    }
    else ROS_WARN("Not clear what to cancel");
    */
}

void GoalCB() {
    const amigo_whole_body_controller::ArmTaskGoal& goal = *add_motion_objective_server_->acceptNewGoal();
    // ToDo: remove objectives
    // ToDo: check if position and orientation constraints have similar link names and root_frame_ids
    // ToDo: keep track of all goals (this most probably means we can't use SIMPLEactionserver stuff)
    // We can keep using the simple action server stuff but in that case cannot keep track of multiple goals at once
    ROS_INFO("Received new motion objective");

    /// If a remove tip frame is present: remove objective
    if (!goal.remove_tip_frame.empty()) {
        std::vector<MotionObjective*> imps_to_remove = wbc->getCartesianImpedances(goal.remove_tip_frame,goal.remove_root_frame);
        for (unsigned int i = 0; i < imps_to_remove.size(); i++) {
            wbc->removeMotionObjective(imps_to_remove[i]);
        }
    }
    /// Else: add motion objectives
    else {
        CartesianImpedance* cartesian_impedance = new CartesianImpedance(goal.position_constraint.link_name);
        geometry_msgs::PoseStamped goal_pose;
        //goal_pose.pose.position.x = goal.position_constraint.position.x;
        //goal_pose.pose.position.y = goal.position_constraint.position.y;
        //goal_pose.pose.position.z = goal.position_constraint.position.z;
        goal_pose.pose.position = goal.position_constraint.position;
        //goal_pose.pose.orientation.x = goal.orientation_constraint.orientation.x;
        //goal_pose.pose.orientation.y = goal.orientation_constraint.orientation.y;
        //goal_pose.pose.orientation.z = goal.orientation_constraint.orientation.z;
        //goal_pose.pose.orientation.w = goal.orientation_constraint.orientation.w;
        goal_pose.pose.orientation = goal.orientation_constraint.orientation;
        goal_pose.header.frame_id = goal.position_constraint.header.frame_id;
        // ToDo: include offset
        cartesian_impedance->setGoal(goal_pose);
        cartesian_impedance->setImpedance(goal.stiffness);
        cartesian_impedance->setPositionTolerance(goal.position_constraint.constraint_region_shape);
        cartesian_impedance->setOrientationTolerance(goal.orientation_constraint.absolute_roll_tolerance, goal.orientation_constraint.absolute_pitch_tolerance, goal.orientation_constraint.absolute_yaw_tolerance);
        if (!wbc->addMotionObjective(cartesian_impedance)) {
            ROS_ERROR("Could not initialize cartesian impedance for new motion objective");
            exit(-1);
        }
    }
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
    n.getParam("/map_3d/resolution", ca_param.environment_collision.octomap_resolution);
}


int main(int argc, char **argv) {

    // Initialize node
    ros::init(argc, argv, "whole_body_controller");
    ros::NodeHandle nh_private("~");

    ros::Subscriber sub_octomap   = nh_private.subscribe<octomap_msgs::OctomapBinary>("/octomap_binary", 10, &octoMapCallback);
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


    // Load parameter files
    CollisionAvoidance::collisionAvoidanceParameters ca_param;
    loadParameterFiles(ca_param);

    wbc = new WholeBodyController(1/loop_rate_);

    ros::Rate r(loop_rate_);

    add_motion_objective_server_ = new action_server(nh_private, "/add_motion_objective", false);
    add_motion_objective_server_->registerGoalCallback(boost::bind(&GoalCB));
    add_motion_objective_server_->registerPreemptCallback(boost::bind(&CancelCB));
    add_motion_objective_server_->start();

    collision_avoidance = new CollisionAvoidance(ca_param, 1/loop_rate_);
    if (!wbc->addMotionObjective(collision_avoidance)) {
        ROS_ERROR("Could not initialize collision avoidance");
        exit(-1);
    }

    ///// Teststuff /////
    /*
    CartesianImpedance* cartesian_impedance;
    cartesian_impedance = new CartesianImpedance("grippoint_left");
    if (!wbc->addMotionObjective(cartesian_impedance)) {
        ROS_ERROR("Could not initialize collision avoidance");
        exit(-1);
    }
    cartesian_impedance = new CartesianImpedance("grippoint_right");
    if (!wbc->addMotionObjective(cartesian_impedance)) {
        ROS_ERROR("Could not initialize collision avoidance");
        exit(-1);
    }
    cartesian_impedance = new CartesianImpedance("grippoint_left");
    if (!wbc->addMotionObjective(cartesian_impedance)) {
        ROS_ERROR("Could not initialize collision avoidance");
        exit(-1);
    }
    int ctr = 0;
    */
    /////////////////////

    //KDL::JntArray q_current;
    Eigen::VectorXd q_ref;
    Eigen::VectorXd qdot_ref;

    while(ros::ok()) {

        ros::spinOnce();

        //wbc->update(q_current, q_ref, qdot_ref);
        wbc->update(q_ref, qdot_ref);

        ///// Teststuff /////
        /*
        std::string root_frame;
        std::vector<MotionObjective*> leftimp = wbc->getCartesianImpedances("grippoint_left",root_frame);
        std::vector<MotionObjective*> rightimp = wbc->getCartesianImpedances("grippoint_right",root_frame);
        ctr++;
        if (ctr == 50) {
            for (unsigned int i = 0; i < leftimp.size(); i++) ROS_INFO("Tip frame: %s\t root frame: %s",leftimp[i]->tip_frame_.c_str(),leftimp[i]->root_frame_.c_str());
            for (unsigned int i = 0; i < rightimp.size(); i++) ROS_INFO("Tip frame: %s\t root frame: %s",rightimp[i]->tip_frame_.c_str(),rightimp[i]->root_frame_.c_str());
            ctr = 0;
        }
        */
        /////////////////////

        //ToDo: set stuff succeeded

        publishJointReferences(wbc->getJointReferences(), wbc->getJointNames());

        r.sleep();
    }

    //ToDo: delete all motion objectives
    delete add_motion_objective_server_;
    delete collision_avoidance;
    delete wbc;

    return 0;
}
