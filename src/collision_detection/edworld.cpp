#include "edworld.h"

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <ros/init.h>
#include <ros/console.h>

namespace wbc {

EdWorld::EdWorld() :
    rate_(10)
{

}

void EdWorld::initialize()
{
    client_.initialize();
    update();
}

void EdWorld::loop()
{
    while (ros::ok())
    {
        ROS_INFO("ed update");
        update();
        rate_.sleep();
    }
}

void EdWorld::start()
{
    thread_ = boost::thread(&EdWorld::loop, this);
}

boost::shared_ptr<fcl::BroadPhaseCollisionManager> EdWorld::getCollisionManager()
{
    boost::lock_guard<boost::mutex> lock(mutex_);
    return world_;
}

void EdWorld::update()
{
    fcl::BroadPhaseCollisionManager *world = client_.getWorld();
    boost::lock_guard<boost::mutex> lock(mutex_);
    world_ = boost::shared_ptr<fcl::BroadPhaseCollisionManager>(world);
}

} // namespace
