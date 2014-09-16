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

    public:
        WholeBodyControllerEdNode(ros::Rate loop_rate);

        void update();

        WholeBodyController wholeBodyController_;

        RobotInterface robot_interface;

        JointTrajectoryAction jte;

    private:
        ros::NodeHandle private_nh;

};

WholeBodyControllerEdNode::WholeBodyControllerEdNode (ros::Rate loop_rate)
    : wholeBodyController_(1/loop_rate.expectedCycleTime().toSec()),
      robot_interface(&wholeBodyController_),
      jte(&wholeBodyController_),
      private_nh("~") {

}

void WholeBodyControllerEdNode::update() {

}

} // namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "whole_body_controller");

    ros::Rate loop_rate(50);

    wbc::WholeBodyControllerEdNode wbcEdNode(loop_rate);

    while (ros::ok()) {
        ros::spinOnce();

        wbcEdNode.update();

        loop_rate.sleep();
    }

    return 0;
}
