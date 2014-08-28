#ifndef VWMCLIENT_H
#define VWMCLIENT_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <wire_volume/transport/Client.h>

#include <geolib_fcl/shapecache.h>

//#include <geolib/Box.h>

#include <fcl/collision_data.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// Timer
#include "profiling/Profiler.h"

namespace vwm_tools {

class vwmClient
{
private:
    vwm::ShapeCache cache;

    std::vector< boost::shared_ptr<fcl::CollisionObject> > world_objects;

    boost::mutex mtx_;
    boost::thread thread_;
    ros::Rate rate_; // 10 hz

public:

    vwmClient();
    ~vwmClient();

    // member variables
    vwm::Client client;
    ros::Publisher marker_pub;

    // member functions
    void update();
    void startThread();
    void loop();

    std::vector< boost::shared_ptr<fcl::CollisionObject> > getWorldObjects();
};

} // namespace

#endif // VWMCLIENT_H
