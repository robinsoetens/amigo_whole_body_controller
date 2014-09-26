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

} // namespace

#endif // WORLD_H
