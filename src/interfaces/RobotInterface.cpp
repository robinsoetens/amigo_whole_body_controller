#include <amigo_whole_body_controller/interfaces/RobotInterface.h>

RobotInterface::RobotInterface(WholeBodyController *wbc)
{
    wbc_ = wbc;
    initialize();
}

RobotInterface::~RobotInterface()
{
    delete torso_pub_;
    delete left_arm_pub_;
    delete right_arm_pub_;
    delete neck_pub_;

}

bool RobotInterface::initialize()
{

    ros::NodeHandle nh_private("~");

    /// Subscribers
    /// Use tf listener in case no pose is published
    bool tf_present = false;
    while (!tf_present)
    {
        ROS_INFO("Waiting for transform from map to base link");
        tf_present = listener_.waitForTransform("/map","/amigo/base_link",ros::Time(0),ros::Duration(1.0)); // Is the latest available transform
    }
    torso_sub_      = nh_private.subscribe<sensor_msgs::JointState>("/amigo/torso/measurements", 1, &RobotInterface::jointMeasurementCallback, this);
    left_arm_sub_   = nh_private.subscribe<sensor_msgs::JointState>("/amigo/left_arm/measurements", 1, &RobotInterface::jointMeasurementCallback, this);
    right_arm_sub_  = nh_private.subscribe<sensor_msgs::JointState>("/amigo/right_arm/measurements", 1, &RobotInterface::jointMeasurementCallback, this);
    neck_sub_       = nh_private.subscribe<sensor_msgs::JointState>("/amigo/neck/measurements", 1, &RobotInterface::jointMeasurementCallback, this);

    /// Publishers
    torso_pub_      = new JointRefPublisher("/amigo/torso/references");
    left_arm_pub_   = new JointRefPublisher("/amigo/left_arm/references");
    right_arm_pub_  = new JointRefPublisher("/amigo/right_arm/references");
    neck_pub_       = new JointRefPublisher("/amigo/neck/references");
    // ToDo: base_pub_

    /// Fill joint_name_to_pub_
    // ToDo: don't hardcode
    joint_name_to_pub_["wrist_yaw_joint_left"] = left_arm_pub_;
    joint_name_to_pub_["wrist_pitch_joint_left"] = left_arm_pub_;
    joint_name_to_pub_["elbow_roll_joint_left"] = left_arm_pub_;
    joint_name_to_pub_["elbow_pitch_joint_left"] = left_arm_pub_;
    joint_name_to_pub_["shoulder_roll_joint_left"] = left_arm_pub_;
    joint_name_to_pub_["shoulder_pitch_joint_left"] = left_arm_pub_;
    joint_name_to_pub_["shoulder_yaw_joint_left"] = left_arm_pub_;
    joint_name_to_pub_["torso_joint"] = torso_pub_;
    joint_name_to_pub_["wrist_yaw_joint_right"] = right_arm_pub_;
    joint_name_to_pub_["wrist_pitch_joint_right"] = right_arm_pub_;
    joint_name_to_pub_["elbow_roll_joint_right"] = right_arm_pub_;
    joint_name_to_pub_["elbow_pitch_joint_right"] = right_arm_pub_;
    joint_name_to_pub_["shoulder_roll_joint_right"] = right_arm_pub_;
    joint_name_to_pub_["shoulder_pitch_joint_right"] = right_arm_pub_;
    joint_name_to_pub_["shoulder_yaw_joint_right"] = right_arm_pub_;
    joint_name_to_pub_["neck_pan_joint"] = neck_pub_;
    joint_name_to_pub_["neck_tilt_joint"] = neck_pub_;

    setAmclPose();

    return true;
}

void RobotInterface::publishJointReferences(const Eigen::VectorXd& joint_refs, const std::vector<std::string>& joint_names) {

    /// Loop over joint_name_to_pub_ map and initialize messages
    // ToDo: can we do this more efficiently (no memory allocation at runtime)
    for(std::map<std::string, JointRefPublisher*>::iterator it_pub = joint_name_to_pub_.begin(); it_pub != joint_name_to_pub_.end(); ++it_pub) {
        it_pub->second->msg_ = sensor_msgs::JointState();
    }

    /// Loop over joint refs and fill in references at correct publisher
    for(unsigned int i = 0; i < joint_refs.size(); ++i) {
        std::map<std::string, JointRefPublisher*>::iterator it_pub = joint_name_to_pub_.find(joint_names[i]);
        it_pub->second->msg_.name.push_back(joint_names[i]);
        it_pub->second->msg_.position.push_back(joint_refs[i]);
        //std::cout << joint_names[i] << ": " << joint_refs[i] << std::endl;
    }

    /// Publish results
    for(std::map<std::string, JointRefPublisher*>::iterator it_pub = joint_name_to_pub_.begin(); it_pub != joint_name_to_pub_.end(); ++it_pub) {
        /// Temporary hack!!!
        if (it_pub->first != "neck_pan_joint" && it_pub->first != "neck_tilt_joint") {
        it_pub->second->pub_.publish(it_pub->second->msg_);
        }
    }
}

void RobotInterface::publishJointTorques(const Eigen::VectorXd& joint_torques, const std::vector<std::string>& joint_names) {

    /// Loop over joint_name_pub_ map and initialize messages
    for(std::map<std::string, JointRefPublisher*>::iterator it_pub = joint_name_to_pub_.begin(); it_pub != joint_name_to_pub_.end(); ++it_pub) {
        it_pub->second->msg_ = sensor_msgs::JointState();
    }

    /// Loop over joint torques and fill in torques at correct publisher
    for(unsigned int i = 0; i < joint_torques.size(); ++i) {
        std::map<std::string, JointRefPublisher*>::iterator it_pub = joint_name_to_pub_.find(joint_names[i]);
        it_pub->second->msg_.name.push_back(joint_names[i]);
        it_pub->second->msg_.effort.push_back(joint_torques[i]);
        //cout << joint_names[i] << ": " << joint_refs[i] << endl;
    }

    /// Publish results
    for(std::map<std::string, JointRefPublisher*>::iterator it_pub = joint_name_to_pub_.begin(); it_pub != joint_name_to_pub_.end(); ++it_pub) {
        it_pub->second->pub_.publish(it_pub->second->msg_);
    }
}

void RobotInterface::setAmclPose()
{
    /// Get transform from base_link to map
    geometry_msgs::PoseStamped base_link_pose, map_pose;
    base_link_pose.header.frame_id = "/amigo/base_link";
    base_link_pose.pose.position.x = 0.0;
    base_link_pose.pose.position.y = 0.0;
    base_link_pose.pose.position.z = 0.0;
    base_link_pose.pose.orientation.x = 0.0;
    base_link_pose.pose.orientation.y = 0.0;
    base_link_pose.pose.orientation.z = 0.0;
    base_link_pose.pose.orientation.w = 1.0;
    try
    {
        listener_.transformPose("/map", base_link_pose, map_pose);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return;
    }

    /// Convert to KDL frame
    KDL::Frame frame;
    frame.p.x(map_pose.pose.position.x);
    frame.p.y(map_pose.pose.position.y);
    frame.p.z(map_pose.pose.position.z);
    frame.M = KDL::Rotation::Quaternion(map_pose.pose.orientation.x,
                                        map_pose.pose.orientation.y,
                                        map_pose.pose.orientation.z,
                                        map_pose.pose.orientation.w);

    /// Set result
    wbc_->robot_state_.setAmclPose(frame);
}

void RobotInterface::jointMeasurementCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for(unsigned int i = 0; i < msg->name.size(); ++i) {
        //ROS_INFO("Setting joint %s to %f",msg->name[i].c_str(), msg->position[i]);
        wbc_->setMeasuredJointPosition(msg->name[i], msg->position[i]);
    }
}
