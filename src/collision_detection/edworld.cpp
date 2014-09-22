#include "edworld.h"

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

namespace wbc {

EdWorld::EdWorld()
{
}

void EdWorld::initialize()
{
    client_.initialize();
    update();
}

boost::shared_ptr<fcl::BroadPhaseCollisionManager> EdWorld::getCollisionManager() const
{
    // TODO: locking
    return world_;
}

void EdWorld::start()
{
    // TODO: start a thread
}

void EdWorld::update()
{
    fcl::BroadPhaseCollisionManager *world = client_.getWorld();
    world_ = boost::shared_ptr<fcl::BroadPhaseCollisionManager>(world);
}

} // namespace
