#ifndef WORLD_H
#define WORLD_H

#include <boost/shared_ptr.hpp>
#include <fcl/broadphase/broadphase.h>

namespace wbc {

class World
{
public:
    World();

    virtual void initialize() {}

    virtual boost::shared_ptr<fcl::BroadPhaseCollisionManager> getCollisionManager() const = 0;

    virtual void start() = 0;
};

} // namespace

#endif // WORLD_H
