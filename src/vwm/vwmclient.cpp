#include "vwm/vwmclient.h"

#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"

#include <wire_volume/EntityHandle.h>
#include <wire_volume/WorldModel.h>

namespace vwm_tools {

vwmClient::vwmClient()
    : cache(), world_objects(), rate_(2)
{
}

vwmClient::~vwmClient()
{
}

void vwmClient::update()
{
    Timer updateTimer;
    updateTimer.start();

    vwm::WorldModelConstPtr world = client.getWorld();

    if (!world) {
        ROS_WARN_ONCE("The world model is not running! Environment collision will not be available");
        return;
    }

    const std::map<vwm::UUID, vwm::EntityHandle>& entities = world->getEntities();

    std::vector< boost::shared_ptr<fcl::CollisionObject> > objects;
    for(std::map<vwm::UUID, vwm::EntityHandle>::const_iterator it = entities.begin(); it != entities.end(); ++it) {
        vwm::EntityHandle e = it->second;

        boost::shared_ptr<fcl::CollisionObject> obj = cache.getCollisionObject(e);

        if (obj) // TODO: check if this holds for shared_ptrs
            objects.push_back(obj);
    }

    mtx_.lock();
    world_objects = objects;
    mtx_.unlock();

    updateTimer.stop();
    ROS_INFO("VWM update took %f ms, (%li/%li) collision bodies found", updateTimer.getElapsedTimeInMilliSec(), objects.size(), entities.size());
}

void vwmClient::startThread()
{
    thread_ = boost::thread(&vwmClient::loop, this);
}

void vwmClient::loop()
{
    while (ros::ok())
    {
        update();
        rate_.sleep();
    }
}

std::vector< boost::shared_ptr<fcl::CollisionObject> > vwmClient::getWorldObjects()
{
    boost::lock_guard<boost::mutex> guard(mtx_);
    return world_objects;
}

} // namespace
