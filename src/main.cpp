#include <amigo_whole_body_controller/wbc_node.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "whole_body_controller");
    ros::NodeHandle nh;

    ros::Rate loop_rate(50);
    wbc::WholeBodyControllerNode wbc_node(loop_rate);

    StatsPublisher sp;
    sp.initialize();

    while (ros::ok()) {
        ros::spinOnce();

        sp.startTimer("main");
        wbc_node.update();
        sp.stopTimer("main");
        sp.publish();

        loop_rate.sleep();
    }

    return 0;
}
