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
private:
    vwm::ShapeCache cache;

    std::vector<fcl::CollisionObject*> world_objects;

public:

    vwmClient();
    ~vwmClient();

    // member variables
    vwm::Client client;
    ros::Publisher marker_pub;

    // member functions
    void update();

    std::vector<fcl::CollisionObject*> getWorldObjects();
};

} // namespace

#endif // VWMCLIENT_H
