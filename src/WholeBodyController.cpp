#include "WholeBodyController.h"

WholeBodyController::WholeBodyController() {

    initialize();

}

WholeBodyController::~WholeBodyController() {

    ROS_INFO("Shutting down whole body controller");
    measured_torso_position_sub_.shutdown();
    measured_left_arm_position_sub_.shutdown();
    measured_right_arm_position_sub_.shutdown();
    measured_head_pan_sub_.shutdown();
    measured_head_tilt_sub_.shutdown();

}

bool WholeBodyController::initialize() {

    // Get node handle
    ros::NodeHandle n("~");
    //ROS_INFO("Nodehandle %s",n.getNamespace().c_str());
    ROS_INFO("Initializing whole body controller");

    // Fill in component description map. This should be parsed from parameter file
    component_description_map_["torso"].root_name = "base";
    component_description_map_["torso"].tip_names.push_back("shoulder_mount_left");
    component_description_map_["torso"].tip_names.push_back("shoulder_mount_right");
    component_description_map_["torso"].number_of_joints = 0;
    component_description_map_["torso"].q.resize(component_description_map_["torso"].number_of_joints);

    component_description_map_["left_arm"].root_name = "shoulder_mount_left";
    component_description_map_["left_arm"].tip_names.push_back("grippoint_left");
    component_description_map_["left_arm"].number_of_joints = 0;
    component_description_map_["left_arm"].q.resize(component_description_map_["left_arm"].number_of_joints);

    component_description_map_["right_arm"].root_name = "shoulder_mount_right";
    component_description_map_["right_arm"].tip_names.push_back("grippoint_right");
    component_description_map_["right_arm"].number_of_joints = 0;
    component_description_map_["right_arm"].q.resize(component_description_map_["right_arm"].number_of_joints);
    /*struct component_description {
            std::string root_name;
            std::string tip_name;
            int number_of_joints;
            int start_index;
            int end_index;*/


    // Implement Jacobian matrix
    // Needs to be done before defining the subscribers, since the callback functions depend on
    // the mapping that results from this initialization
    ComputeJacobian_.Initialize(&component_description_map_);
    ///ROS_INFO("Number of torso joints equals %i", component_description_map_["torso"].number_of_joints);
    ///ROS_INFO("Number of left arm joints equals %i", component_description_map_["left_arm"].number_of_joints);
    ///ROS_INFO("Number of right arm joints equals %i", component_description_map_["right_arm"].number_of_joints);
    q_current_.resize(ComputeJacobian_.num_joints);
    // ToDo: Make number of columns variable
    Jacobian_.resize(12,ComputeJacobian_.num_joints);

    // Initialize subscribers etc.
    setTopics();

    // Implement left Cartesian Impedance
    CIleft_.initialize();

    ROS_INFO("Whole Body Controller Initialized");

    return true;

}

bool WholeBodyController::update() {

    ///ROS_INFO("Updating wholebodycontroller");
    ComputeJacobian_.Update(component_description_map_, q_current_, Jacobian_);
    ROS_INFO("Eighth column of returned Jacobian is \n%f\n%f\n%f\n%f\n%f\n%f",Jacobian_(0,7),Jacobian_(1,7),Jacobian_(2,7),Jacobian_(3,7),Jacobian_(4,7),Jacobian_(5,7));
    ///ROS_INFO("Updated wholebodycontroller");

    return true;

}

void WholeBodyController::setTopics() {

    // Get node handle
    ros::NodeHandle n("~");

    std::vector<double> temp_vector;
    // ToDo: standardize data types joint positions
    // ToDo: make temp_vector.resize and map arguments variable
    // ToDo: fill in odom topic
    //odom_sub_ = ;
    measured_torso_position_sub_ = n.subscribe<std_msgs::Float64>("/spindle_position", 1, &WholeBodyController::callbackMeasuredTorsoPosition, this);
    temp_vector.resize(1);
    q_current_map_["spindle_joint"] = temp_vector;

    measured_left_arm_position_sub_ = n.subscribe<amigo_msgs::arm_joints>("/arm_left_controller/joint_measurements", 1, &WholeBodyController::callbackMeasuredLeftArmPosition, this);
    temp_vector.resize(7);
    q_current_map_["shoulder_yaw_joint_left"] = temp_vector;

    measured_right_arm_position_sub_ = n.subscribe<amigo_msgs::arm_joints>("/arm_right_controller/joint_measurements", 1, &WholeBodyController::callbackMeasuredRightArmPosition, this);
    temp_vector.resize(7);
    q_current_map_["shoulder_yaw_joint_right"] = temp_vector;

    measured_head_pan_sub_ = n.subscribe<std_msgs::Float64>("/head_pan_angle", 1, &WholeBodyController::callbackMeasuredHeadPan, this);
    temp_vector.resize(1);
    q_current_map_["pan_joint"] = temp_vector; //NOT_CORRECT

    measured_head_tilt_sub_ = n.subscribe<std_msgs::Float64>("/head_tilt_angle", 1, &WholeBodyController::callbackMeasuredHeadTilt, this);
    temp_vector.resize(1);
    q_current_map_["tilt_joint"] = temp_vector; //NOT CORRECT



}

void WholeBodyController::callbackMeasuredTorsoPosition(const std_msgs::Float64::ConstPtr& msg) {
    /*
    //TODO: Get rid of hardcoded root_joint
    std::string root_joint = "spindle_joint";

    std::vector<int> index_vector = ComputeJacobian_.index_map[root_joint];

    for (int i = 0; i < (1 + index_vector[1] - index_vector[0]); i++) {
        q_current_(index_vector[0]+i) = msg->data;
        //ROS_INFO("Joint %i = %f", index_vector[0]+i, msg->data);
    }

    for (uint i = 0; i<q_current_map_[root_joint].size(); i++) {
        q_current_map_[root_joint][i] = msg->data;
        //ROS_INFO("%s %i = %f", root_joint.c_str(), i+1, q_current_map_[root_joint][i]);
    }
     */
    std::string component_name = "torso";
    for (int i = 0; i<component_description_map_[component_name].number_of_joints; i++) {
        component_description_map_[component_name].q[i] = msg->data;
        //ROS_INFO("%s joint %i = %f",component_name.c_str(),i+1,component_description_map_[component_name].q[i]);
    }

}

void WholeBodyController::callbackMeasuredLeftArmPosition(const amigo_msgs::arm_joints::ConstPtr& msg) {
    /*
    //TODO: Get rid of hardcoded root_joint
    std::string root_joint = "shoulder_yaw_joint_left";

    std::vector<int> index_vector = ComputeJacobian_.index_map[root_joint];

    for (int i = 0; i < (1 + index_vector[1] - index_vector[0]); i++) {
        q_current_(index_vector[0]+i) = msg->pos[i].data;
        //ROS_INFO("Joint %i = %f", index_vector[0]+i, msg->pos[i].data);
    }
    for (uint i = 0; i<q_current_map_[root_joint].size(); i++) {
        q_current_map_[root_joint][i] = msg->pos[i].data;
        //ROS_INFO("%s %i = %f", root_joint.c_str(), i+1, q_current_map_[root_joint][i]);
    }
     */
    std::string component_name = "left_arm";
    uint num_comp_joints = component_description_map_[component_name].number_of_joints;
    for (uint i = 0; i<num_comp_joints; i++) {
        component_description_map_[component_name].q[i] = msg->pos[i].data;

        // I guess we need to reverse this vector --> probably not
        ///component_description_map_[component_name].q[i] = msg->pos[num_comp_joints-1-i].data;
        //ROS_INFO("%s joint %i = %f",component_name.c_str(),i+1,component_description_map_[component_name].q[i]);
    }
}

void WholeBodyController::callbackMeasuredRightArmPosition(const amigo_msgs::arm_joints::ConstPtr& msg) {
    /*
    //TODO: Get rid of hardcoded root_joint
    std::string root_joint = "shoulder_yaw_joint_right";

    std::vector<int> index_vector = ComputeJacobian_.index_map[root_joint];

    for (int i = 0; i < (1 + index_vector[1] - index_vector[0]); i++) {
        q_current_(index_vector[0]+i) = msg->pos[i].data;
        //ROS_INFO("Joint %i = %f", index_vector[0]+i, msg->pos[i].data);
    }
    for (uint i = 0; i<q_current_map_[root_joint].size(); i++) {
        q_current_map_[root_joint][i] = msg->pos[i].data;
        //ROS_INFO("%s %i = %f", root_joint.c_str(), i+1, q_current_map_[root_joint][i]);
    }
     */
    std::string component_name = "right_arm";
    uint num_comp_joints = component_description_map_[component_name].number_of_joints;
    for (uint i = 0; i<num_comp_joints; i++) {
        component_description_map_[component_name].q[i] = msg->pos[i].data;

        // I guess we need to reverse this vector --> probably not
        ///component_description_map_[component_name].q[i] = msg->pos[num_comp_joints-1-i].data;
        //ROS_INFO("%s joint %i = %f",component_name.c_str(),i+1,component_description_map_[component_name].q[i]);
    }
}

void WholeBodyController::callbackMeasuredHeadPan(const std_msgs::Float64::ConstPtr& msg) {

}

void WholeBodyController::callbackMeasuredHeadTilt(const std_msgs::Float64::ConstPtr& msg) {

}

//TEMP
/*
void WholeBodyController::callbackTarget(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    ROS_INFO("Received target");
    //Transform message into tooltip frame. This directly implies e = x_d - x
    errorPose = WholeBodyController::transformPose(listener, *msg);
    ROS_DEBUG("errorpose = %f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%f",errorPose.pose.position.x,errorPose.pose.position.y,errorPose.pose.position.z,errorPose.pose.orientation.x,errorPose.pose.orientation.y,errorPose.pose.orientation.z,errorPose.pose.orientation.w);

}

//TEMP
geometry_msgs::PoseStamped WholeBodyController::transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped poseMsg){

    geometry_msgs::PoseStamped poseInGripperFrame;

    listener.transformPose("/grippoint_left",poseMsg,poseInGripperFrame);

    return poseInGripperFrame;

}*/
