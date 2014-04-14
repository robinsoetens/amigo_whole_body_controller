#ifndef VWMCLIENT_H
#define VWMCLIENT_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <wire_volume/transport/Client.h>

#include <geolib/Shape.h>

#include <wire_volume/UpdateRequest.h>
#include <wire_volume/transport/Serializer.h>
#include <wire_volume/Entity.h>
#include <wire_volume/WorldModel.h>

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


    // member functions
    void initRandomObj();
    void update();
};

} // namespace

#endif // VWMCLIENT_H
