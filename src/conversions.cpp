#include "conversions.h"

#include <fcl/BVH/BVH_model.h>

namespace wbc {

void poseFCLToTF(const fcl::Transform3f in, tf::Pose &out)
{
    const fcl::Quaternion3f q = in.getQuatRotation();
    const fcl::Vec3f        t = in.getTranslation();

    out = tf::Pose(
                tf::Quaternion(q.getX(), q.getY(), q.getZ(), q.getW()),
                tf::Vector3(t[0], t[1], t[2]));
}

void poseFCLToMsg(const fcl::Transform3f in, geometry_msgs::Pose &out)
{
    tf::Pose pose;
    poseFCLToTF(in, pose);
    tf::poseTFToMsg(pose, out);
}

void poseTFToFCL(const tf::Pose in, fcl::Transform3f &out)
{
    tf::Vector3 pos = in.getOrigin();
    tf::Quaternion q = in.getRotation();
    out.setTranslation(fcl::Vec3f(pos.getX(), pos.getY(), pos.getZ()));
    out.setQuatRotation(fcl::Quaternion3f(q.getW(), q.getX(), q.getY(), q.getZ()));
}

void pointFCLToTF(const fcl::Vec3f in, tf::Point &out)
{
    //out(in[0], in[1], in[2]);
    out = tf::Point(in[0], in[1], in[2]);
}

void pointFCLToMsg(const fcl::Vec3f in, geometry_msgs::Point &out)
{
    tf::Point p;
    pointFCLToTF(in, p);
    tf::pointTFToMsg(p, out);
}

void getFclShape(const fcl::CollisionGeometry *cg, std::vector<fcl::Vec3f> &vertices, std::vector<fcl::Triangle> &triangles)
{
    vertices.clear();
    triangles.clear();

    fcl::NODE_TYPE nodeType = cg->getNodeType();
    fcl::OBJECT_TYPE objType = cg->getObjectType();

    switch(objType)
    {
    case fcl::OT_BVH: {
        switch (nodeType)
        {
        case fcl::BV_OBBRSS: {
            const fcl::BVHModel<fcl::OBBRSS>* model = dynamic_cast<const fcl::BVHModel<fcl::OBBRSS>*>(cg);
            //fcl::Vec3f* temp = new fcl::Vec3f[model->num_vertices];
            //memcpy(temp, model->vertices, sizeof(fcl::Vec3f) * model->num_vertices);

            vertices  = std::vector<fcl::Vec3f>(   model->vertices,    model->vertices    + model->num_vertices);
            triangles = std::vector<fcl::Triangle>(model->tri_indices, model->tri_indices + model->num_tris);

            break;
        }
        default: {
            ROS_WARN_ONCE("error converting unknown fcl node type: %i", nodeType);
            break;
        }
        }
        break;
    }
    default:
    {
        ROS_WARN_ONCE("error converting unknown fcl object type: %i", objType);
        break;
    }
    }
}

void objectFCLtoMarker(const fcl::CollisionObject &obj, visualization_msgs::Marker &triangle_list)
{
    triangle_list.type = visualization_msgs::Marker::TRIANGLE_LIST;
    triangle_list.action = visualization_msgs::Marker::ADD;

    triangle_list.header.stamp = ros::Time::now();

    poseFCLToMsg(obj.getTransform(), triangle_list.pose);

    triangle_list.scale.x = 1.0;
    triangle_list.scale.y = 1.0;
    triangle_list.scale.z = 1.0;

    std::vector<fcl::Vec3f>    vertices;
    std::vector<fcl::Triangle> triangles;
    getFclShape(obj.getCollisionGeometry(), vertices, triangles);

    std::vector<fcl::Triangle>::iterator it;
    for (it = triangles.begin(); it < triangles.end(); it++)
    {
        fcl::Triangle t = *it;
        geometry_msgs::Point p0, p1, p2;

        pointFCLToMsg(vertices[t[0]], p0);
        pointFCLToMsg(vertices[t[1]], p1);
        pointFCLToMsg(vertices[t[2]], p2);

        triangle_list.points.push_back(p0);
        triangle_list.points.push_back(p1);
        triangle_list.points.push_back(p2);
    }
}

}
