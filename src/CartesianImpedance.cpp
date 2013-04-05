#include "CartesianImpedance.h"
#include <tf/transform_datatypes.h>

#include <amigo_whole_body_controller/ArmTaskAction.h>
#include <actionlib/server/simple_action_server.h>

CartesianImpedance::CartesianImpedance() {

}

CartesianImpedance::~CartesianImpedance() {

    delete server_;

}

bool CartesianImpedance::initialize(const std::string& end_effector_frame, uint F_start_index) {

    ROS_INFO("Initializing Cartesian Impedance");

    // Get node handle
    ros::NodeHandle n("~");
    std::string ns = ros::this_node::getName();
    //ROS_INFO("Nodehandle %s",n.getNamespace().c_str());

    end_effector_frame_ = end_effector_frame;
    F_start_index_ = F_start_index;
    is_active_ = false;

    // String to indicate which side to take for subscriber and action client
    std::string side;
    if (!std::strcmp(end_effector_frame.c_str(),"/grippoint_left")) side = "left";
    else if (!std::strcmp(end_effector_frame.c_str(),"/grippoint_right")) side = "right";
    else ROS_ERROR("Argument $SIDE argument unknown");
    ROS_INFO("Side = %s",side.c_str());

    /*if (end_effector_frame_ == "/grippoint_left") {
        target_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/arm_left_target_pose", 10, &CartesianImpedance::callbackTarget, this);
    }
    else if (end_effector_frame_ == "/grippoint_right") {
        target_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/arm_right_target_pose", 10, &CartesianImpedance::callbackTarget, this);
    }*/
    target_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/arm_"+side+"_target_pose", 10, &CartesianImpedance::callbackTarget, this);
    ROS_INFO("Subscribed to %s topic for target",target_sub_.getTopic().c_str());

    // Fill in impedance matrix
    // ToDo: Parametrize
    K_.resize(6,6);
    //double standard_factor = 25;
    for (uint i = 0; i<6; i++) {
        for (uint ii = 0; ii<6; ii++) {
            K_(i,ii) = 0;
            ///if (i != ii) K_(i,ii) = 0;
            ///else K_(i,ii) = standard_factor;
        }
    }
    n.param<double> (ns+"/kx", K_(0,0), 26);
    n.param<double> (ns+"/ky", K_(1,1), 26);
    n.param<double> (ns+"/kz", K_(2,2), 26);
    n.param<double> (ns+"/krx", K_(3,3), 26);
    n.param<double> (ns+"/kry", K_(4,4), 26);
    n.param<double> (ns+"/krz", K_(5,5), 26);
    ///ROS_INFO("K = %f, %f, %f, %f, %f, %f",K_(0,0),K_(1,1),K_(2,2),K_(3,3),K_(4,4),K_(5,5));

    error_vector_.resize(6);
    for (uint i = 0; i<6; i++) error_vector_(i) = 0;

    // Wait for transform
    //ToDo: make this work!
    bool wait_for_transform = listener.waitForTransform(end_effector_frame_,"/map", ros::Time::now(), ros::Duration(1.0));
    if (!wait_for_transform) ROS_WARN("Transform between %s and /map is not available",end_effector_frame.c_str());

    /////
    ///KDL::ChainIkSolverVel_pinv* ik_solver_vel; //hpp
    ///ik_solver_vel = new KDL::ChainIkSolverVel_pinv(chain);//cpp
    ///action_server* server_; //h
    /*server_ = new action_server(n, "/grasp_precompute_left", boost::bind(&execute, _1, &server, &client), false);
    action_server_(node_, "joint_trajectory_action",
                           boost::bind(&JointTrajectoryExecuter::goalCB, this, _1),
                           boost::bind(&JointTrajectoryExecuter::cancelCB, this, _1),
                           false)
    */

    server_ = new action_server(n, "/grasp_precompute_" + side, false);
    server_->registerGoalCallback(boost::bind(&CartesianImpedance::goalCB, this));
    server_->registerPreemptCallback(boost::bind(&CartesianImpedance::cancelCB, this));

    server_->start();

    // Pre-grasping
    pre_grasp_ = false;
    n.param<double> (ns+"/pre_grasp_delta", pre_grasp_delta_, 0.1);
    /////

    ROS_INFO("Initialized Cartesian Impedance");

    return true;

}

void CartesianImpedance::callbackTarget(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //////
    /*
    ROS_INFO("Received target");

    is_active_ = true;

    //Transform message into tooltip frame. This directly implies e = x_d - x
    /////errorPose = CartesianImpedance::transformPose(listener, *msg);
    errorPose = CartesianImpedance::transformPose(*msg);

    // ToDo: Check orientations as well
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(errorPose.pose.orientation, orientation);
    double roll, pitch, yaw;
    btMatrix3x3(orientation).getRPY(roll, pitch, yaw);
    error_vector_(0) = errorPose.pose.position.x;
    error_vector_(1) = errorPose.pose.position.y;
    error_vector_(2) = errorPose.pose.position.z;
    error_vector_(3) = roll;
    error_vector_(4) = pitch;
    error_vector_(5) = yaw;

    ROS_INFO("errorpose = %f,\t%f,\t%f,\t%f,\t%f,\t%f",error_vector_(0),error_vector_(1),error_vector_(2),error_vector_(3),error_vector_(4),error_vector_(5));
    */
    //////

}

/////geometry_msgs::PoseStamped CartesianImpedance::transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped poseMsg){
geometry_msgs::PoseStamped CartesianImpedance::transformPose(geometry_msgs::PoseStamped poseMsg){

    //OBSOLETE??//

    geometry_msgs::PoseStamped poseInGripperFrame;
    /*
    // Wait for transform??
    listener.transformPose(end_effector_frame_,poseMsg,poseInGripperFrame);
*/
    return poseInGripperFrame;

}

void CartesianImpedance::update(Eigen::VectorXd& F_task, uint& force_vector_index) {

    /// Previous implementation listening to topics
    /*ROS_INFO("Updating Cartesian Impedance, force_vector_index = %i",force_vector_index);

    if (is_active_) {
        F_task.segment(force_vector_index,6) = K_ * error_vector_;
        force_vector_index += 6;
    }
    ///else for (uint i = 0; i < 6; i++) F_task(i+force_vector_index) = 0;

    is_active_ = false;*/

    /////
    if (is_active_) {

        //Transform message into tooltip frame. This directly implies e = x_d - x
        /////errorPose = CartesianImpedance::transformPose(listener, goal_pose_);
        //errorPose = CartesianImpedance::transformPose(goal_pose_);

        //listener.transformPose(end_effector_frame_,goal_pose_,errorPose);

        // ToDo: why doesn't the state above work? Seems more sensible than below in terms of realtime behavior

        //listener.waitForTransform()

        //ros::Time now = ros::Time::now();
        //bool wait_for_transform = listener.waitForTransform(end_effector_frame_,goal_pose_.header.frame_id, now, ros::Duration(1.0));

        goal_pose_.header.stamp = ros::Time();
        try {
            listener.transformPose(end_effector_frame_, goal_pose_, errorPose);
        } catch (tf::TransformException& e) {
            ROS_ERROR("CartesianImpedance: %s", e.what());
            return;
        }

        //listener.transformPose(end_effector_frame_,now,goal_pose_,goal_pose_.header.frame_id,errorPose);

        // ToDo: Check orientations as well
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(errorPose.pose.orientation, orientation);
        double roll, pitch, yaw;

        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        error_vector_(0) = errorPose.pose.position.x;
        error_vector_(1) = errorPose.pose.position.y;
        error_vector_(2) = errorPose.pose.position.z;
        error_vector_(3) = roll;
        error_vector_(4) = pitch;
        error_vector_(5) = yaw;

        // If pre-grasping, an offset is added to the errorpose in x-direction
        if (pre_grasp_) error_vector_(0) -= pre_grasp_delta_;

        ///ROS_INFO("errorpose = %f,\t%f,\t%f,\t%f,\t%f,\t%f",error_vector_(0),error_vector_(1),error_vector_(2),error_vector_(3),error_vector_(4),error_vector_(5));

        F_task.segment(force_vector_index,6) = K_ * error_vector_;
        force_vector_index += 6;

        // ToDo: Include boundaries in action message
        int num_converged_dof = 0;
        for (uint i = 0; i < 3; i++) {
            if (fabs(error_vector_(i)) < 0.035) ++num_converged_dof;
        }
        for (uint i = 0; i < 3; i++) {
            if (fabs(error_vector_(i+3)) < 0.3) ++num_converged_dof;
        }
        if (num_converged_dof == 6 && pre_grasp_) {
            pre_grasp_ = false;
            ROS_INFO("pre_grasp_ = %d",pre_grasp_);
        }
        else if (num_converged_dof == 6 && server_->isActive() && !pre_grasp_) {
            server_->setSucceeded();
            ROS_WARN("errorpose = %f,\t%f,\t%f,\t%f,\t%f,\t%f",error_vector_(0),error_vector_(1),error_vector_(2),error_vector_(3),error_vector_(4),error_vector_(5));
        }
    }
    /////

}

/////
void CartesianImpedance::goalCB() {

    const amigo_arm_navigation::grasp_precomputeGoal& goal = *server_->acceptNewGoal();

    ROS_WARN("Received new goal");

    /*
    // Cancels the currently active goal.
    if (active_goal_.getGoalStatus().status == 1) {
        // Stop something???

        // Marks the current goal as canceled.
        active_goal_.setCanceled();
        is_active_ = false;
        ROS_WARN("Canceling previous goal");
    }
    */

    // ToDo: Check whether the received goal meets it requirements
    /*if (gh.getGoal()->PERFORM_PRE_GRASP) {
        ROS_ERROR("Pre-grasp not yet implemented, rejecting goal");
        gh.setRejected();
    }*/
    //else {

    /// Hack: set Cartesian impedance to inactive if frame_id equals "reset"
    if (goal.goal.header.frame_id == "reset") {
        is_active_ = false;
        server_->setSucceeded();
        return;
    }

    /// Put the goal data in the right datatype
    /// If it's a delta goal this will be corrected further on
    goal_pose_.header = goal.goal.header;
    goal_pose_.pose.position.x = goal.goal.x;
    goal_pose_.pose.position.y = goal.goal.y;
    goal_pose_.pose.position.z = goal.goal.z;
    double roll = goal.goal.roll;
    double pitch = goal.goal.pitch;
    double yaw = goal.goal.yaw;
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    goal_pose_.pose.orientation = orientation;

    // Check for absolute or delta (and ambiqious goals)
    double EPS = 1e-6;
    bool absolute_requested=false, delta_requested=false;
    if(fabs(goal.goal.x)>EPS || fabs(goal.goal.y)>EPS || fabs(goal.goal.z)>EPS || fabs(goal.goal.roll)>EPS || fabs(goal.goal.pitch)>EPS || fabs(goal.goal.yaw)>EPS)
        absolute_requested = true;
    if(fabs(goal.delta.x)>EPS || fabs(goal.delta.y)>EPS || fabs(goal.delta.z)>EPS || fabs(goal.delta.roll)>EPS || fabs(goal.delta.pitch)>EPS || fabs(goal.delta.yaw)>EPS)
        delta_requested = true;
    if(absolute_requested && delta_requested)
    {
        server_->setAborted();
        ROS_WARN("grasp_precompute_action: goal consists out of both absolute AND delta values. Choose only one!");
        return;
    }

    /// Check if a delta is requested
    // ToDo: can't we use transformpose?
    if(delta_requested)
    {
        ROS_DEBUG("Delta requested of x=%f \t y=%f \t z=%f \t roll=%f \t pitch=%f \t yaw=%f",goal.delta.x,goal.delta.y,goal.delta.z,goal.delta.roll,goal.delta.pitch,goal.delta.yaw);
        // Create tmp variable
        tf::StampedTransform tmp;

        // Lookup the current vector of the desired TF to the grippoint
        if(listener.waitForTransform(goal.delta.header.frame_id, end_effector_frame_, goal.delta.header.stamp, ros::Duration(1.0))) {
            try
            {listener.lookupTransform(goal.delta.header.frame_id,end_effector_frame_, goal.delta.header.stamp, tmp);
            }
            catch (tf::TransformException ex){
                server_->setAborted();
                ROS_ERROR("%s",ex.what());
                return;}
        }
        else {
            server_->setAborted();
            ROS_ERROR("grasp_precompute_action: TF_LISTENER could not find transform from gripper to %s:",goal.delta.header.frame_id.c_str());
            return;

        }

        // Print the transform
        ROS_DEBUG("tmp.x=%f \t tmp.y=%f \t tmp.z=%f tmp.X=%f \t tmp.Y=%f \t tmp.Z=%f \t tmp.W=%f",tmp.getOrigin().getX(),tmp.getOrigin().getY(),tmp.getOrigin().getZ(),tmp.getRotation().getX(),tmp.getRotation().getY(),tmp.getRotation().getZ(),tmp.getRotation().getW());

        // Add the delta values and overwrite input variable
        ROS_INFO("Child frame id=%s", tmp.frame_id_.c_str());
        goal_pose_.header.frame_id  = tmp.frame_id_;
        goal_pose_.header.stamp     = tmp.stamp_;
        goal_pose_.pose.position.x  = tmp.getOrigin().getX() + goal.delta.x;
        goal_pose_.pose.position.y  = tmp.getOrigin().getY() + goal.delta.y;
        goal_pose_.pose.position.z  = tmp.getOrigin().getZ() + goal.delta.z;
        double roll, pitch, yaw;
        tmp.getBasis().getRPY(roll, pitch, yaw);
        goal_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll + goal.delta.roll, pitch + goal.delta.pitch, yaw + goal.delta.yaw);
    }

    // If input frame is not /base_link, convert to base_link
    if(goal_pose_.header.frame_id.compare("/base_link"))
    {
        ROS_DEBUG("Frame id was not BASE_LINK");
        // Create tmp variable
        geometry_msgs::PoseStamped tmp;

        // Perform the transformation to /base_link
        if(listener.waitForTransform("/base_link", goal_pose_.header.frame_id, goal_pose_.header.stamp, ros::Duration(1.0))) {
            try
            {listener.transformPose("/base_link", goal_pose_, tmp);}
            catch (tf::TransformException ex){
                server_->setAborted();
                ROS_ERROR("%s",ex.what());
                return;}
        }
        else {
            server_->setAborted();
            ROS_ERROR("grasp_precompute_action: TF_LISTENER could not find transform from /base_link to %s:",goal_pose_.header.frame_id.c_str());
            return;

        }
        /// Overwrite input values
        goal_pose_ = tmp;
    }

    // Pre-grasp
    if (goal.PERFORM_PRE_GRASP) {
        pre_grasp_ = true;
        ROS_INFO("pre_grasp_ = %d",pre_grasp_);
    }

    // Set goal to accepted etc;
    //gh.setAccepted();
    //active_goal_ = gh;
    is_active_ = true;
    //}



}

void CartesianImpedance::cancelCB() {

    /*
    if (active_goal_ == gh)
    {
        // Marks the current goal as canceled.
        active_goal_.setCanceled();
        is_active_ = false;
    }
    */

    is_active_ = false;
    server_->setPreempted();
}

/////
