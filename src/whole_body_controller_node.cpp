#include "WholeBodyController.h"
#include <amigo_whole_body_controller/interfaces/RobotInterface.h>
#include <amigo_whole_body_controller/interfaces/JointTrajectoryAction.h>
#include <amigo_whole_body_controller/ArmTaskAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// tf
#include <tf/transform_listener.h>

using namespace std;

const double loop_rate_ = 50;

typedef actionlib::SimpleActionServer<amigo_whole_body_controller::ArmTaskAction> action_server;
action_server* add_motion_objective_server_;
CollisionAvoidance* collision_avoidance;
CartesianImpedance* cartesian_impedance;
//RobotState* robot_state;

WholeBodyController* wbc;

#if ROS_VERSION_MINIMUM(1,9,0)
// Groovy
void octoMapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    if(tree){
        octomap::OcTreeStamped* octree = dynamic_cast<octomap::OcTreeStamped*>(tree);
        if(!octree){
            ROS_ERROR("No Octomap created");
        }
        else{
            collision_avoidance->setOctoMap(octree);
        }
        delete tree;
    }
    else{
        ROS_ERROR("Octomap conversion error");
        exit(1);
    }
}
#elif ROS_VERSION_MINIMUM(1,8,0)
// Fuerte
void octoMapCallback(const octomap_msgs::OctomapBinary::ConstPtr& msg)
{
    octomap::AbstractOcTree* octree;
    octree = octomap_msgs::binaryMsgDataToMap(msg->data);
    std::stringstream datastream;
    //ROS_INFO("Writing data to stream");
    octree->writeData(datastream);
    //octree->readBinaryData(datastream);
    if (octree) {
        octomap::OcTreeStamped* octreestamped;
        octreestamped = new octomap::OcTreeStamped(0.05);
        //ROS_INFO("Reading data from stream");
        octreestamped->readData(datastream);
        //ROS_INFO("Read data from stream");
        //octreestamped = dynamic_cast<octomap::OcTreeStamped*>(octree);
        if (!octreestamped){
            ROS_ERROR("No Octomap created");
        }
        else{
            collision_avoidance->setOctoMap(octreestamped);
        }
        delete octree;
    }
    else{
        ROS_ERROR("Octomap conversion error");
        exit(1);
    }

}
#endif

void CancelCB() {
    ROS_INFO("Canceling goal");
    add_motion_objective_server_->setPreempted();
    // ToDo: remove motion objective
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
        std::vector<MotionObjective*> imps_to_remove = wbc->getCartesianImpedances(goal.position_constraint.link_name,goal.position_constraint.header.frame_id);
        for (unsigned int i = 0; i < imps_to_remove.size(); i++) {
            wbc->removeMotionObjective(imps_to_remove[i]);
        }

        cartesian_impedance = new CartesianImpedance(goal.position_constraint.link_name);
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.pose.position = goal.position_constraint.position;
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
#if ROS_VERSION_MINIMUM(1,9,0)
    // Groovy
    ros::Subscriber sub_octomap   = nh_private.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, &octoMapCallback);
#elif ROS_VERSION_MINIMUM(1,8,0)
    // Fuerte
    ros::Subscriber sub_octomap   = nh_private.subscribe<octomap_msgs::OctomapBinary>("/octomap_binary", 10, &octoMapCallback);
#endif

    // Load parameter files
    CollisionAvoidance::collisionAvoidanceParameters ca_param;
    loadParameterFiles(ca_param);

    // Determine whether to publish torques or position references
    bool omit_admittance = false;
    std::string ns = ros::this_node::getName();
    nh_private.param<bool> (ns+"/omit_admittance", omit_admittance, true);
    ROS_WARN("Omit admittance = %d",omit_admittance);

    /// Whole body controller object
    wbc = new WholeBodyController(1/loop_rate_);

    /// Robot interface
    RobotInterface robot_interface(wbc);

    /// Joint trajectory executer
    JointTrajectoryAction jte(wbc);

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

    std::string root_frame;

    while(ros::ok()) {

        ros::spinOnce();

        // Beun oplossing
        std::vector<MotionObjective*> left_imp = wbc->getCartesianImpedances("grippoint_left", root_frame);
        std::vector<MotionObjective*> right_imp = wbc->getCartesianImpedances("grippoint_right", root_frame);
        if (!left_imp.empty()){
            if (cartesian_impedance->getStatus() == 1 && add_motion_objective_server_->isActive()){

                add_motion_objective_server_->setSucceeded();
                //ROS_INFO("Impedance status %i, Tip frame: %s root frame: %s",cartesian_impedance->getStatus(),left_imp[0]->tip_frame_.c_str(),left_imp[0]->root_frame_.c_str());
            }
        }
        if (!right_imp.empty()){
            if (cartesian_impedance->getStatus() == 1 && add_motion_objective_server_->isActive()){

                add_motion_objective_server_->setSucceeded();
                //ROS_INFO("Impedance status %i, Tip frame: %s root frame: %s",cartesian_impedance->getStatus(),right_imp[0]->tip_frame_.c_str(),right_imp[0]->root_frame_.c_str());
            }
        }

        /// Set base pose in whole-body controller (remaining joints are set implicitly in the callback functions in robot_interface)
        robot_interface.setAmclPose();

        /// Update whole-body controller
        Eigen::VectorXd q_ref, qdot_ref;
        wbc->update(q_ref, qdot_ref);

        /// Update the joint trajectory executer
        jte.update();

        //ToDo: set stuff succeeded
        if (!omit_admittance)
        {
            ROS_WARN_ONCE("Publishing reference positions");
            robot_interface.publishJointReferences(wbc->getJointReferences(), wbc->getJointNames());
        }
        else
        {
            ROS_WARN_ONCE("Publishing reference torques");
            robot_interface.publishJointTorques(wbc->getJointTorques(), wbc->getJointNames());
        }

        r.sleep();
    }
    //ToDo: delete all motion objectives
    delete add_motion_objective_server_;
    delete collision_avoidance;
    delete wbc;
    delete cartesian_impedance;

    return 0;
}
