#include "WholeBodyController.h"

// tf
#include <tf/transform_listener.h>

const double loop_rate_ = 50;

Constraint* cart_imp_left;
Constraint* cart_imp_right;

int main(int argc, char **argv) {

    // Initialize node
    ros::init(argc, argv, "whole_body_controller");
    ros::NodeHandle nh_private("~");

    ros::Rate r(loop_rate_);

    WholeBodyController wbc;

    tf::TransformListener tf_listener;

    CartesianImpedance* cart_imp_left = new CartesianImpedance("/grippoint_left", &tf_listener);
    wbc.addConstraint(cart_imp_left);

    CartesianImpedance* cart_imp_right = new CartesianImpedance("/grippoint_right", &tf_listener);
    wbc.addConstraint(cart_imp_right);

    tf::Stamped<tf::Pose> left_goal;
    cart_imp_left->setGoal(left_goal);

    while(ros::ok()) {

        ros::spinOnce();

        wbc.update();

        r.sleep();
    }

    return 0;
}
