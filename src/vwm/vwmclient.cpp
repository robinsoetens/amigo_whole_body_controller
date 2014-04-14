#include "vwm/vwmclient.h"

#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"

#include "vwm/vwm_tools.h"
#include <wire_volume/EntityHandle.h>

#include "profiling/Profiler.h"

namespace vwm_tools {

vwmClient::vwmClient()
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
        std::cout << "  " << e.getID() << std::endl;

        vwm::TransformPtr t;
        if (!e.getTransformFrom("/world", t)) {
            std::cout << "    No transform found" << std::endl;
            continue;
        }

        geo::Pose3D pose;
        if (t->getBestPose(pose)) {
            std::cout << "    " << pose << std::endl;
        }

        geo::ShapeConstPtr shape = e.getShape();
        if (!shape) {
            std::cout << "    no shape" << std::endl;
            continue;
        }

        std::cout << "    shape: " << shape->getMesh().getTriangles().size() << " triangles" << std::endl;

        fcl::CollisionObject obj = vwm_tools::transform2fcl(shape, pose);

        //DistanceData cdata = distance(main_obj, obj);

        //fcl::Vec3f v1 = cdata.result.nearest_points[0];
        //fcl::Vec3f v2 = cdata.result.nearest_points[1];
        //drawLine(v1, v2, e.getID());
    }

    updateTimer.stop();
    ROS_INFO("VWM update took %f ms", updateTimer.getElapsedTimeInMilliSec());
}

} // namespace
