/*!
 * \author Paul Metsemakers
 * \date January, 2013
 * \version 0.1
 */

#ifndef WBC_OBSTACLEAVOIDANCE_H_
#define WBC_OBSTACLEAVOIDANCE_H_

// ROS
#include "ros/ros.h"

// Eigen
#include <Eigen/Core>

// Plugins
#include "MotionObjective.h"
#include "ChainParser.h"
#include "Chain.h"
#include "Tree.h"

// Map
#include <tue_map_3d/Map3D.h>


// Bullet GJK Closest Point calculation
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h>
#include <Bullet-C-Api.h>

/////

class CollisionAvoidance : public MotionObjective
{

    struct Box
    {
        Box(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
            : min_(min), max_(max)
        {
        }
        Eigen::Vector3d min_;
        Eigen::Vector3d max_;
    };


public:

    /// Define the type of OctoMap as timestamped
    typedef octomap::OcTreeStamped OctreeType;

    struct collisionAvoidanceParameters
    {
        struct Parameters
        {
            double f_max;
            double d_threshold;
            int order;
            double octomap_resolution;
        } ;
        Parameters self_collision;
        Parameters environment_collision;
    } ca_param_;

    //ToDo: make configure, start- and stophook. Components can be started/stopped in an actionlib kind of fashion

    /**
     * Constructor
     */
    CollisionAvoidance(collisionAvoidanceParameters &parameters, const double Ts);

    /**
     * Deconstructor
     */
    virtual ~CollisionAvoidance();

    /**
     * Initialize function
     */
    bool initialize(RobotState &robotstate);

    void apply(RobotState& robotstate);

    void setOctoMap(const octomap_msgs::OctomapBinary&  octomap_msg);

protected:

    //! Sampling time
    double Ts_;

    Chain* chain_left_;
    Chain* chain_right_;

    Tree tree_;

    RobotState* robot_state_;

    ros::Publisher pub_model_marker_;
    ros::Publisher pub_forces_marker_;
    ros::Publisher pub_bbx_marker_;

    std::vector< std::vector<RobotState::CollisionBody> > active_groups_;
    std::vector< std::vector<RobotState::CollisionBody> > collision_groups_;

    struct Voxel {
        geometry_msgs::PoseStamped center_point;
        double size_voxel;
    };
    struct Distance {
        std::string frame_id;
        btPointCollector bt_distance;
    } ;

    struct ReactionForce {
        std::string frame_id;
        btVector3 pointOnA;
        btVector3 direction;
        btScalar amplitude;
    } ;

    struct Wrench {
        std::string frame_id;
        Eigen::VectorXd wrench;
    } ;

    geometry_msgs::PoseStamped no_fix_;

    //octomap::OcTreeStamped* octomap_;
    octomap::OcTree* octomap_;

    // Minimum and maximum point of the BBX
    std::vector<octomath::Vector3> min_;
    std::vector<octomath::Vector3> max_;

    void getposeRPY(geometry_msgs::PoseStamped& pose, Eigen::Vector3d& RPY);

    void calculateWrenches(std::vector<ReactionForce> &reaction_forces, std::vector<Wrench> &wrenches_out);

    void selfCollision(std::vector<Distance> &min_distances, std::vector<ReactionForce> &reaction_forces);

    void environmentCollision(std::vector<Distance> &min_distances, std::vector<ReactionForce> &reaction_forces);

    void initializeCollisionModel(RobotState &robotstate);

    void calculateTransform();

    void setTransform(btTransform& transform_out, geometry_msgs::PoseStamped &fkPose, geometry_msgs::PoseStamped& fixPose);

    void distanceCalculation(btConvexShape &shapeA, btConvexShape &shapeB, btTransform& transformA, btTransform& transformB, btPointCollector& distance_out);

    void pickMinimumDistance(std::vector<Distance> &calculatedDistances, std::vector<Distance> &minimumDistances);

    void calculateReactionForce(std::vector<Distance> &minimumDistances, std::vector<ReactionForce> &reactionForces, collisionAvoidanceParameters::Parameters &param);

    void bulletTest();

    void outputWrenches(std::vector<Wrench> &wrenches);

    void visualize(std::vector<Distance> &min_distances) const;

    void visualizeCollisionModel(RobotState::CollisionBody collisionBody,int id)  const;

    void visualizeReactionForce(Distance &d_min, int id) const;

    void visualizeBBX(octomath::Vector3 min, octomath::Vector3 max, int id) const;

    void findOuterPoints(RobotState::CollisionBody& collisionBody, btVector3 &min, btVector3 &max);

};

#endif
