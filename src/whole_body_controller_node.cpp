#include "WholeBodyController.h"

const double loop_rate_ = 50;

int main(int argc, char **argv) {

    // Initialize node
    ros::init(argc, argv, "whole_body_controller");
    ros::NodeHandle nh_private("~");

    ros::Rate r(loop_rate_);

    WholeBodyController wbc;

    while(ros::ok()) {

        ros::spinOnce();

        wbc.update();

        r.sleep();
    }

    return 0;
}
