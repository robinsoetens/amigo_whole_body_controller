#ifndef VWMCLIENT_H
#define VWMCLIENT_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <wire_volume/transport/Client.h>

#include <geolib_fcl/shapecache.h>

#include <geolib/Box.h>

#include <fcl/collision_data.h>

namespace vwm_tools {

class vwmClient
{
public:
    // constructor
    vwmClient();
    ~vwmClient();

    // member variables
    vwm::Client client;
    ros::Publisher marker_pub;

    vwm::ShapeCache cache;

    // member functions
    void initRandomObj();
    void update();
};

} // namespace

#endif // VWMCLIENT_H
