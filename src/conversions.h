#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <fcl/collision_object.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

namespace wbc {



void poseFCLToTF(const fcl::Transform3f in, tf::Pose &out);

void poseFCLToMsg(const fcl::Transform3f in, geometry_msgs::Pose &out);

void poseTFToFCL(const tf::Pose in, fcl::Transform3f &out);

void pointFCLToTF(const fcl::Vec3f in, tf::Point &out);

void pointFCLToMsg(const fcl::Vec3f in, geometry_msgs::Point &out);

void getFclShape(const fcl::CollisionGeometry *cg, std::vector<fcl::Vec3f> &vertices, std::vector<fcl::Triangle> &triangles);

void objectFCLtoMarker(const fcl::CollisionObject &obj, visualization_msgs::Marker &triangle_list);



}
#endif // CONVERSIONS_H
