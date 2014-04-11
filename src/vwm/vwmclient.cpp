#include "vwm/vwmclient.h"

#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"

#include "vwm/vwm_tools.h"

namespace vwm_tools {

struct TStruct
{
  std::vector<double> records;
  double overall_time;

  TStruct() { overall_time = 0; }

  void push_back(double t)
  {
    records.push_back(t);
    overall_time += t;
  }
};

vwmClient::vwmClient()
{
}

vwmClient::~vwmClient()
{
}

void vwmClient::initRandomObj()
{
    // put some stuff in the VWM

    vwm::UpdateRequest req;
    req.createEntity("test-entity");

    geo::ShapePtr shape(new geo::Box(geo::Vector3(-0.1, -0.1, -0.1), geo::Vector3(0.1, 0.1, 0.1)));
    req.setShape("test-entity", shape);

    req.addTransform("/world", "test-entity", geo::Pose3D(1, 0, 2, 0.8, 0.4, 0.2));

    client.sendUpdateRequest(req);
}

void vwmClient::step()
{
    vwm::WorldModelConstPtr world = client.getWorld();

    const std::map<vwm::UUID, vwm::EntityHandle>& entities = world->getEntities();

    vwm::EntityHandle mainEntity = world->getEntity("test-entity");

    if (!mainEntity.exists())
    {
        ROS_INFO("No main entity found");
        return;
    }

    vwm::TransformPtr main_t;
    if (!mainEntity.getTransformFrom("/world", main_t)) {
        std::cout << "    No transform found" << std::endl;
        return;
    }

    geo::Pose3D main_pose;
    if (main_t->getBestPose(main_pose)) {
        std::cout << "    " << main_pose << std::endl;
    }

    geo::ShapeConstPtr main_shape = mainEntity.getShape();
    if (!main_shape) {
        std::cout << "    no shape" << std::endl;
        return;
    }

    fcl::CollisionObject main_obj = vwm_tools::transform2fcl(main_shape, main_pose);

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
}

void vwmClient::drawLine(fcl::Vec3f v1, fcl::Vec3f v2, std::string ns)
{
    visualization_msgs::Marker line_strip;

    line_strip.header.frame_id = "/map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = ns;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    line_strip.scale.x = 0.01;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    geometry_msgs::Point p1;
    p1.x = v1.data[0];
    p1.y = v1.data[1];
    p1.z = v1.data[2];
    line_strip.points.push_back(p1);
    geometry_msgs::Point p2;
    p2.x = v2.data[0];
    p2.y = v2.data[1];
    p2.z = v2.data[2];
    line_strip.points.push_back(p2);

    marker_pub.publish(line_strip);
}

} // namespace
