#include "WholeBodyController.h"
#include "WholeBodyController.h"
#include <amigo_whole_body_controller/interfaces/RobotInterface.h>
#include <amigo_whole_body_controller/interfaces/JointTrajectoryAction.h>
#include <amigo_whole_body_controller/ArmTaskAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// tf
#include <tf/transform_listener.h>
#include <octomap_msgs/conversions.h>
#include <profiling/StatsPublisher.h>

// connection to ed
#include <ed_wbc/ed_client.h>


namespace wbc {

class WholeBodyControllerEdNode {

    protected:
        // nodehandle must be created before everything else
        ros::NodeHandle private_nh;

        wbc::CollisionAvoidance::collisionAvoidanceParameters loadCollisionAvoidanceParameters() const;

        void GoalCB();

        void CancelCB();

    public:
        WholeBodyControllerEdNode(ros::Rate &loop_rate);

        WholeBodyController wholeBodyController_;

        RobotInterface robot_interface;

        JointTrajectoryAction jte;

        /// Action server for adding/removing cartesian impedance goals
        actionlib::SimpleActionServer<amigo_whole_body_controller::ArmTaskAction> add_motion_objective_server_;

        /// Motion objectives

        CollisionAvoidance::collisionAvoidanceParameters ca_param;
        CollisionAvoidance collision_avoidance;

        /// Determine whether to publish torques or position references */
        bool omit_admittance;

        /// main loop
        void update();

};

WholeBodyControllerEdNode::WholeBodyControllerEdNode (ros::Rate &loop_rate)
    : private_nh("~"),
      wholeBodyController_(loop_rate.expectedCycleTime().toSec()),
      robot_interface(&wholeBodyController_),
      jte(&wholeBodyController_),
      add_motion_objective_server_(private_nh, "/add_motion_objective", false),
      ca_param(loadCollisionAvoidanceParameters()),
      collision_avoidance(ca_param, loop_rate.expectedCycleTime().toSec()),
      omit_admittance(false)
{
    add_motion_objective_server_.registerGoalCallback(
        boost::bind(&WholeBodyControllerEdNode::GoalCB, this)
    );
    add_motion_objective_server_.registerPreemptCallback(
        boost::bind(&WholeBodyControllerEdNode::CancelCB, this)
    );

    //add_motion_objective_server_->registerPreemptCallback(boost::bind(&CancelCB));
    add_motion_objective_server_.start();

    if (!wholeBodyController_.addMotionObjective(&collision_avoidance)) {
        ROS_ERROR("Could not initialize collision avoidance");
        exit(-1);
    }

    // get omit_admittance from the parameter server
    std::string ns = ros::this_node::getName();
    private_nh.param<bool> (ns+"/omit_admittance", omit_admittance, true);
    ROS_WARN("Omit admittance = %d", omit_admittance);

    // Before startup: make sure all joint values are initialized properly
    bool initcheck = false;
    ros::spinOnce();
    while (!initcheck && ros::ok()) {
        ros::spinOnce();
        robot_interface.setAmclPose();
        initcheck = robot_interface.isInitialized();
        if (!initcheck) ROS_INFO("Waiting for all joints to be initialized");
        loop_rate.sleep();
    }
}

wbc::CollisionAvoidance::collisionAvoidanceParameters WholeBodyControllerEdNode::loadCollisionAvoidanceParameters() const {
    wbc::CollisionAvoidance::collisionAvoidanceParameters ca_param;
    ros::NodeHandle n("~");
    std::string ns = ros::this_node::getName();
    n.param<double> (ns+"/collision_avoidance/self_collision/F_max", ca_param.self_collision.f_max, 1.0);
    n.param<double> (ns+"/collision_avoidance/self_collision/d_threshold", ca_param.self_collision.d_threshold, 1.0);
    n.param<int> (ns+"/collision_avoidance/self_collision/order", ca_param.self_collision.order, 1);

    n.param<double> (ns+"/collision_avoidance/environment_collision/F_max", ca_param.environment_collision.f_max, 1.0);
    n.param<double> (ns+"/collision_avoidance/environment_collision/d_threshold", ca_param.environment_collision.d_threshold, 1.0);
    n.param<int> (ns+"/collision_avoidance/environment_collision/order", ca_param.environment_collision.order, 1);
    n.getParam("/map_3d/resolution", ca_param.environment_collision.octomap_resolution);

    return ca_param;
}

void WholeBodyControllerEdNode::GoalCB() {
    ROS_INFO("GoalCB");
}

void WholeBodyControllerEdNode::CancelCB() {
    ROS_INFO("Canceling goal");
    add_motion_objective_server_.setPreempted();
    // ToDo: remove motion objective
}


void WholeBodyControllerEdNode::update() {

    // Set base pose in whole-body controller (remaining joints are set implicitly in the callback functions in robot_interface)
    robot_interface.setAmclPose();

    // Update whole-body controller
    Eigen::VectorXd q_ref, qdot_ref;

    wholeBodyController_.update(q_ref, qdot_ref);

    // Update the joint trajectory executer
    jte.update();

    // ToDo: set stuff succeeded
    if (!omit_admittance)
    {
        ROS_WARN_ONCE("Publishing reference positions");
        robot_interface.publishJointReferences(wholeBodyController_.getJointReferences(), wholeBodyController_.getJointNames());
        // ROS_ERROR_ONCE("NO REFERENCES PUBLISHED");
    }
    else
    {
        ROS_WARN_ONCE("Publishing reference torques");
        robot_interface.publishJointTorques(wholeBodyController_.getJointTorques(), wholeBodyController_.getJointNames());
    }
}

} // namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "whole_body_controller");
    ros::NodeHandle nh;

    ros::Rate loop_rate(50);
    wbc::WholeBodyControllerEdNode wbcEdNode(loop_rate);

    StatsPublisher sp;
    sp.initialize();

    while (ros::ok()) {
        ros::spinOnce();

        sp.startTimer("main");
        wbcEdNode.update();
        sp.stopTimer("main");
        sp.publish();

        loop_rate.sleep();
    }

    return 0;
}
