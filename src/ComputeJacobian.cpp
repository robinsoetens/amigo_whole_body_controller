#include "ComputeJacobian.h"

ComputeJacobian::ComputeJacobian() {

}

ComputeJacobian::~ComputeJacobian() {

}

bool ComputeJacobian::Initialize() {

    //Load parameters from parameter server
    // Get node handle
    ros::NodeHandle n("~");

    if (!n.getParam("chain_array", chain_array)) {
        ROS_FATAL("Cannot load chain array from parameter server");
        return false;
    }
    //(Display results)

    // For every entry in array:
    //          Make kinematic chain
    //          Resize joint arrays
    //          Do bookkeeping

    return true;
}
