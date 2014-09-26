#ifndef WORLDCLIENT_H
#define WORLDCLIENT_H

#include "world.h"

namespace wbc {

class WorldClient
{
public:
    WorldClient();

    virtual void initialize() {}

    virtual World* getWorld() = 0;

    virtual void start() = 0;
};

} // namespace

#endif // WORLDCLIENT_H
