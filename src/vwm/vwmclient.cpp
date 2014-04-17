#include "vwm/vwmclient.h"

#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"

#include <wire_volume/EntityHandle.h>
#include <wire_volume/WorldModel.h>

#include "profiling/Profiler.h"

namespace vwm_tools {

vwmClient::vwmClient()
    : cache()
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

    const std::map<vwm::UUID, vwm::EntityHandle>& entities = world->getEntities();

    for(std::map<vwm::UUID, vwm::EntityHandle>::const_iterator it = entities.begin(); it != entities.end(); ++it) {
        vwm::EntityHandle e = it->second;
        std::cout << "  entity: " << e.getID() << std::endl;

        fcl::CollisionObject* obj = cache.getCollisionObject(e);

        if (obj) {
            std::cout << "    made a collision object" << std::endl;
        } else {
            std::cout << "    error, no collision object made" << std::endl;
        }

    }

    updateTimer.stop();
    ROS_INFO("VWM update took %f ms", updateTimer.getElapsedTimeInMilliSec());
}

} // namespace
