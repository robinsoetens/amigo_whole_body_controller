#include "amigo_whole_body_controller/wbc_node.h"

namespace wbc {

WholeBodyControllerNode::WholeBodyControllerNode (ros::Rate &loop_rate)
    : private_nh("~"),
      loop_rate_(loop_rate),
      wholeBodyController_(loop_rate.expectedCycleTime().toSec()),
      robot_interface(&wholeBodyController_),
      jte(&wholeBodyController_),
      motion_objective_server_(private_nh, "/add_motion_objective", false),
      ca_param(loadCollisionAvoidanceParameters()),
      collision_avoidance(ca_param, loop_rate.expectedCycleTime().toSec()),
      world_client_(0),
      omit_admittance(false)
{
    motion_objective_server_.registerGoalCallback(
        boost::bind(&WholeBodyControllerNode::goalCB, this, _1)
    );
    motion_objective_server_.registerCancelCallback(
        boost::bind(&WholeBodyControllerNode::cancelCB, this, _1)
    );

    motion_objective_server_.start();

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

void WholeBodyControllerNode::setCollisionWorld(WorldClient *world_client)
{
    world_client_ = world_client;
    collision_avoidance.setCollisionWorld(world_client);

    world_client_->initialize();
    world_client_->start();
}

wbc::CollisionAvoidance::collisionAvoidanceParameters WholeBodyControllerNode::loadCollisionAvoidanceParameters() const {
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

void WholeBodyControllerNode::goalCB(MotionObjectiveServer::GoalHandle handle) {
    std::string id = handle.getGoalID().id;
    ROS_INFO("GoalCB: %s", id.c_str());

    MotionObjectiveServer::GoalConstPtr goal = handle.getGoal();

    CartesianImpedance *cartesian_impedance = new CartesianImpedance(goal->position_constraint.link_name, loop_rate_.expectedCycleTime().toSec());
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose.position = goal->position_constraint.position;
    goal_pose.pose.orientation = goal->orientation_constraint.orientation;
    goal_pose.header.frame_id = goal->position_constraint.header.frame_id;
    // ToDo: include offset
    cartesian_impedance->setGoal(goal_pose);
    cartesian_impedance->setGoalOffset(goal->position_constraint.target_point_offset);
    cartesian_impedance->setImpedance(goal->stiffness);
    cartesian_impedance->setPositionTolerance(goal->position_constraint.constraint_region_shape);
    cartesian_impedance->setOrientationTolerance(goal->orientation_constraint.absolute_roll_tolerance, goal->orientation_constraint.absolute_pitch_tolerance, goal->orientation_constraint.absolute_yaw_tolerance);

    if (!wholeBodyController_.addMotionObjective(cartesian_impedance)) {
        ROS_ERROR("Could not initialize cartesian impedance for new motion objective");
        exit(-1);
    }

    // add it to the map
    goal_map[id] = std::pair<MotionObjectiveServer::GoalHandle, MotionObjectivePtr>(handle, MotionObjectivePtr(cartesian_impedance));
    handle.setAccepted();
}

void WholeBodyControllerNode::cancelCB(MotionObjectiveServer::GoalHandle handle) {
    std::string id = handle.getGoalID().id;
    ROS_INFO("cancelCB: %s", id.c_str());

    // delete from the WBC
    MotionObjectivePtr motion_objective = goal_map[id].second;
    wholeBodyController_.removeMotionObjective(motion_objective.get());

    // delete from the map
    if (1 != goal_map.erase(id)) {
        ROS_WARN("could not find %s in the goal_map", id.c_str());
    }
}

void WholeBodyControllerNode::update() {

    // check for succeeded motion objectives
    // using a variation of Mark Ransom algorithm (http://stackoverflow.com/a/180772)
    for(GoalMotionObjectiveMap::iterator it = goal_map.begin(); it != goal_map.end();)
    {
        MotionObjectiveServer::GoalHandle handle = it->second.first;
        MotionObjectivePtr motion_objective      = it->second.second;

        if (motion_objective->getStatus() == 1) // status == done
        {
            wholeBodyController_.removeMotionObjective(motion_objective.get());
            handle.setSucceeded();
            goal_map.erase(it++);  // Note the post increment here.
                                   // This increments 'it' and returns a copy of
                                   // the original 'it' to be used by erase()
        }
        else
        {
            ++it;  // Use Pre-Increment here as it is more effecient
                   // Because no copy of it is required.
        }
    }

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
