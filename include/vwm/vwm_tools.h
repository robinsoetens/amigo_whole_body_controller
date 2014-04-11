#ifndef WIRE_TOOLS_H
#define WIRE_TOOLS_H

#include <geolib/Shape.h>

#include <wire_volume/UpdateRequest.h>
#include <wire_volume/transport/Serializer.h>
#include <wire_volume/Entity.h>
#include <wire_volume/WorldModel.h>

#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"

namespace vwm_tools {

inline fcl::CollisionObject transform2fcl(const geo::ShapeConstPtr &shape, const geo::Pose3D &pose)
{
    // cast to tf::Pose
    tf::Pose tf = pose;

    // convert tf to fcl transform
    tf::Vector3 pos = tf.getOrigin();
    tf::Quaternion q = tf.getRotation();
    fcl::Transform3f fcl_transform;
    fcl_transform.setTranslation(fcl::Vec3f(pos.getX(), pos.getY(), pos.getZ()));
    fcl_transform.setQuatRotation(fcl::Quaternion3f(q.getW(), q.getX(), q.getY(), q.getZ()));

    // get the obj's mesh
    geo::Mesh mesh = shape->getMesh();

    // convert it to fcl mesh
    std::vector<tf::Vector3>   ps = mesh.getPoints();
    std::vector<geo::TriangleI> ts = mesh.getTriangleIs();

    std::vector<fcl::Vec3f>    fcl_ps;
    std::vector<fcl::Triangle> fcl_ts;

    for(std::vector<tf::Vector3>::iterator it = ps.begin(); it != ps.end(); ++it)
    {
        tf::Vector3 v = *it;
        fcl_ps.push_back(fcl::Vec3f(v.getX(), v.getY(), v.getZ()));
    }

    for(std::vector<geo::TriangleI>::iterator it = ts.begin(); it != ts.end(); ++it)
    {
        geo::TriangleI t = *it;
        fcl_ts.push_back(fcl::Triangle(t.i1_, t.i2_, t.i3_));
    }

    // BVHModel is a template class for mesh geometry, for default OBBRSS template is used
    //typedef fcl::BVHModel<fcl::OBBRSS>* Model;

    fcl::BVHModel<fcl::OBBRSS>* model = new fcl::BVHModel<fcl::OBBRSS>();

    model->beginModel();
    model->addSubModel(fcl_ps, fcl_ts);
    model->endModel();

    // construct the CollisionObject

    boost::shared_ptr<fcl::CollisionGeometry> model_ptr(model);

    fcl::CollisionObject obj(model_ptr, fcl_transform);
    return obj;
}

/*
inline void transform2fcl(const Eigen::Affine3d &b, fcl::Transform3f &f)
{
  Eigen::Quaterniond q(b.rotation());
  f.setTranslation(fcl::Vec3f(b.translation().x(), b.translation().y(), b.translation().z()));
  f.setQuatRotation(fcl::Quaternion3f(q.w(), q.x(), q.y(), q.z()));
}

inline fcl::Transform3f transform2fcl(const Eigen::Affine3d &b)
{
  fcl::Transform3f t;
  transform2fcl(b, t);
  return t;
}
*/

} // namespace

#endif // WIRE_TOOLS_H
