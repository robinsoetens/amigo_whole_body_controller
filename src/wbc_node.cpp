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

        void update();

        WholeBodyController wholeBodyController_;

        RobotInterface robot_interface;

        JointTrajectoryAction jte;

        actionlib::SimpleActionServer<amigo_whole_body_controller::ArmTaskAction> add_motion_objective_server_;

        // Motion objectives
        CollisionAvoidance::collisionAvoidanceParameters ca_param;
        CollisionAvoidance collision_avoidance;

    private:


};

WholeBodyControllerEdNode::WholeBodyControllerEdNode (ros::Rate &loop_rate)
    : private_nh("~"),
      wholeBodyController_(1/loop_rate.expectedCycleTime().toSec()),
      robot_interface(&wholeBodyController_),
      jte(&wholeBodyController_),
      add_motion_objective_server_(private_nh, "/add_motion_objective", false),
      ca_param(loadCollisionAvoidanceParameters()),
      collision_avoidance(ca_param, 0.5)
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

}

} // namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "whole_body_controller");
    ros::NodeHandle nh;

    ros::Rate loop_rate(50);
    wbc::WholeBodyControllerEdNode wbcEdNode(loop_rate);

    while (ros::ok()) {
        ros::spinOnce();

        wbcEdNode.update();

        loop_rate.sleep();
    }

    return 0;
}
