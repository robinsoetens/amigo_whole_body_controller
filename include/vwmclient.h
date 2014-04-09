#ifndef VWMCLIENT_H
#define VWMCLIENT_H

#define USE_FCL

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
#include "fcl_utility.h"

namespace wire_tools {

class vwmClient
{
public:
    // constructor
    vwmClient();
    ~vwmClient();

    // member variables
    vwm::Client client;
    ros::Publisher marker_pub;

#ifndef USE_FCL
    btConvexPenetrationDepthSolver*	depthSolver;
    btSimplexSolverInterface* simplexSolver;
#endif

    // member functions
    void initRandomObj();
    void step();
    DistanceData distance(fcl::CollisionObject obj1, fcl::CollisionObject obj2);
    void drawLine(Vec3f v1, Vec3f v2, std::string ns);

#ifndef USE_FCL
    void distanceCalculation(btConvexShape& shapeA, btConvexShape& shapeB, btTransform& transformA, btTransform& transformB, btPointCollector &distance_out);
#endif
};

} // namespace

#endif // VWMCLIENT_H
