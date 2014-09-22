#ifndef EDWORLD_H
#define EDWORLD_H

#include "world.h"
#include <ed_wbc/ed_client.h>
#include <boost/thread.hpp>
#include <ros/rate.h>

namespace wbc {

class EdWorld : public World
{
  public:

    EdWorld();

    void initialize();

    void start();

    boost::shared_ptr<fcl::BroadPhaseCollisionManager> getCollisionManager() const;

  protected:

    ros::Rate rate_;

    void loop();

    void update();

    boost::thread thread_;

    boost::mutex mtx_;

    boost::shared_ptr<fcl::BroadPhaseCollisionManager> world_;

    ed_wbc::EdClient client_;

};

} // namespace

#endif // EDWORLD_H
