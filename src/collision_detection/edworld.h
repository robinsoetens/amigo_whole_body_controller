#ifndef EDWORLD_H
#define EDWORLD_H

#include "world.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <ed_wbc/ed_client.h>
#include <ros/rate.h>

namespace wbc {

class EdWorld : public World
{
  public:

    EdWorld();

    void initialize();

    void start();

    boost::shared_ptr<fcl::BroadPhaseCollisionManager> getCollisionManager();

  protected:

    ros::Rate rate_;

    void loop();

    void update();

    boost::thread thread_;

    boost::mutex mutex_;

    boost::shared_ptr<fcl::BroadPhaseCollisionManager> world_;

    ed_wbc::EdClient client_;

};

} // namespace

#endif // EDWORLD_H
