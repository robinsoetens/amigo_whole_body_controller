#include "WholeBodyController.h"

double loop_rate_ = 10;

int main(int argc, char **argv) {

    // Initialize node
    ros::init(argc, argv, "WholeBodyController");
    ros::NodeHandle nh_private("~");

    ros::Rate r(loop_rate_);

    while(ros::ok()) {

        ros::spinOnce();

        r.sleep();
    }

    return 0;
}
