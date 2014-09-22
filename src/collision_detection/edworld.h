#ifndef EDWORLD_H
#define EDWORLD_H

#include "world.h"
#include <ed_wbc/ed_client.h>

namespace wbc {

class EdWorld : public World
{
  public:
    EdWorld();

    void initialize();

    boost::shared_ptr<fcl::BroadPhaseCollisionManager> getCollisionManager() const;

    void start();

  protected:

    void update();

    boost::shared_ptr<fcl::BroadPhaseCollisionManager> world_;

    ed_wbc::EdClient client_;

};

} // namespace

#endif // EDWORLD_H
