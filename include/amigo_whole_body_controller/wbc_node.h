#include <profiling/StatsPublisher.h>
#include <amigo_whole_body_controller/interfaces/RobotInterface.h>
#include <amigo_whole_body_controller/interfaces/JointTrajectoryAction.h>
#include <amigo_whole_body_controller/ArmTaskAction.h>

#include "amigo_whole_body_controller/motionobjectives/CollisionAvoidance.h"
#include "amigo_whole_body_controller/motionobjectives/CartesianImpedance.h"

#include "WholeBodyController.h"
#include "amigo_whole_body_controller/world.h"

#include <boost/shared_ptr.hpp>

namespace wbc {

class WholeBodyControllerNode {

  protected:
    // nodehandle must be created before everything else
    ros::NodeHandle private_nh;

  public:
    WholeBodyControllerNode(ros::Rate &loop_rate);

    ros::Rate &loop_rate_;

    WholeBodyController wholeBodyController_;

    RobotInterface robot_interface;

    JointTrajectoryAction jte;

    /// Action server for adding/removing cartesian impedance goals
    typedef actionlib::ActionServer<amigo_whole_body_controller::ArmTaskAction> MotionObjectiveServer;
    MotionObjectiveServer motion_objective_server_;

    typedef std::map<std::string, std::pair<MotionObjectiveServer::GoalHandle, MotionObjectivePtr> > GoalMotionObjectiveMap;
    GoalMotionObjectiveMap goal_map;

    /// Motion objectives

    CollisionAvoidance::collisionAvoidanceParameters ca_param;
    CollisionAvoidance collision_avoidance;

    /// Connection to ED (the world model)
    World *world_;

    /// Determine whether to publish torques or position references */
    bool omit_admittance;

    /// main loop
    void update();

    void setCollisionWorld(World *world);

  protected:
    wbc::CollisionAvoidance::collisionAvoidanceParameters loadCollisionAvoidanceParameters() const;

    std::vector< boost::shared_ptr<MotionObjective> > motion_objectives_;

    void goalCB(MotionObjectiveServer::GoalHandle handle);

    void cancelCB(MotionObjectiveServer::GoalHandle handle);

};

} // namespace
