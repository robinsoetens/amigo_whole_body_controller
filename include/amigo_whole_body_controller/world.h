#ifndef WORLD_H
#define WORLD_H

#include <boost/shared_ptr.hpp>
#include <fcl/broadphase/broadphase.h>

namespace wbc {

class World
{
public:
    World();

    virtual fcl::BroadPhaseCollisionManager* getCollisionManager() = 0;

};

typedef boost::shared_ptr<World> WorldPtr;
typedef boost::shared_ptr<const World> WorldConstPtr;

} // namespace

#endif // WORLD_H
