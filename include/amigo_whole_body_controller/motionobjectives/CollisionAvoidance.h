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

#ifdef USE_BULLET
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
#endif

#ifdef USE_FCL

// VWM stuff
#include <vwm/vwmclient.h>

// FCL closest point
#include <fcl/distance.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>

#endif

// Timer
#include "profiling/Profiler.h"

/////

// ToDo: why not make total wrenches and total distances member variables? Now they're passed on from function to function

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

    void setOctoMap(octomap::OcTreeStamped* octree);
    
    void removeOctomapBBX(const geometry_msgs::Point& goal, const std::string& root);

    /** Adds object collisionmodel to robot collisionmodel
     * @param frame_id: frame where the object is added to */
    void addObjectCollisionModel(const std::string& frame_id);

    /** Remove object collision model
     */
    void removeObjectCollisionModel();


protected:

    //! Sampling time
    double Ts_;

    Tree tree_;

    RobotState* robot_state_;

    ros::Publisher pub_model_marker_;
    ros::Publisher pub_model_marker_fcl_;
    ros::Publisher pub_forces_marker_;
    ros::Publisher pub_forces_marker_fcl_;
    ros::Publisher pub_bbx_marker_;

    struct Voxel {
        KDL::Frame center_point;
        double size_voxel;
    };
    struct Distance {
        std::string frame_id;
        btPointCollector bt_distance;
    } ;
    struct Distance2 {
        std::string frame_id;
#ifdef USE_FCL
        fcl::DistanceResult result;
#endif
    };

    struct RepulsiveForce {
        std::string frame_id;
        btVector3 pointOnA;
        btVector3 direction;
        btScalar amplitude;
    } ;

    struct RepulsiveForce2 {
        std::string frame_id;
        Eigen::Vector3d pointOnA;
        Eigen::Vector3d direction;
        float amplitude;
    };

    // provide << printing functionality
    friend std::ostream& operator << (std::ostream &o, const RepulsiveForce &r)
    {
        o << r.frame_id << " ";
        o << r.pointOnA[0]  << " " << r.pointOnA[1]  << " " << r.pointOnA[2]  << " ";
        o << r.direction[0] << " " << r.direction[1] << " " << r.direction[2];
        return o;
    }

    // provide << printing functionality
    friend std::ostream& operator << (std::ostream &o, const RepulsiveForce2 &r)
    {
        o << r.frame_id << " ";
        o << r.pointOnA[0]  << " " << r.pointOnA[1]  << " " << r.pointOnA[2]  << " ";
        o << r.direction[0] << " " << r.direction[1] << " " << r.direction[2];
        return o;
    }

    struct Wrench {
        std::string frame_id;
        Eigen::VectorXd wrench;
    } ;

    /** Matrix containing the combined Jacobian matrix of this objective */
    Eigen::MatrixXd jacobian_pre_alloc_;
    /** Vector containing all relevant wrench entries */
    Eigen::VectorXd wrenches_pre_alloc_;

    KDL::Frame no_fix_;

    octomap::OcTreeStamped* octomap_;

#ifdef USE_FCL
    vwm_tools::vwmClient client_;
#endif

    btConvexPenetrationDepthSolver*	depthSolver;
    btSimplexSolverInterface* simplexSolver;

    // Minimum and maximum point of the BBX
    std::vector<octomath::Vector3> min_;
    std::vector<octomath::Vector3> max_;

    // debugging timers
    Timer timer_total;
    Timer timer_boost;
    Timer timer_fcl;
    Timer timer_octomap;

    double time_total;
    double time_boost;
    double time_fcl;
    double time_octomap;

    int report_counter;

    /**
     * @brief Calculate the repulsive forces as a result of self collision avoidance
     * @param Output: Vector with the minimum distances between robot collision bodies, vector with the repulsive forces
     */
    void selfCollision(std::vector<Distance> &min_distances, std::vector<Distance2> &min_distances2);

    /**
     * @brief Calculate the repulsive forces as a result of environment collision avoidance
     * @param Output: Vector with the minimum distances to the environment, vector with the repulsive forces
     */
    void environmentCollision(   std::vector<Distance>  &min_distances, std::vector<RepulsiveForce> &repulsive_forces);
#ifdef USE_FCL
    void environmentCollisionVWM(std::vector<Distance2> &min_distances);
#endif

    /**
     * @brief Construct the collision bodies
     * @param Input: The robot state
     */
    void initializeCollisionModel(RobotState &robotstate);

    /**
     * @brief Calculate the pose of the collision bodies
     */
    void calculateTransform();

    /**
     * @brief Calculate the Bullet shape pose from the corresponding FK pose using a correction from this frame pose
     * @param Input: Pose of the KDL frame and the fix for the collision bodies, Output: Bullet transform
     */
#ifdef USE_BULLET
    void setTransform(const KDL::Frame &fkPose, const KDL::Frame& fixPose, btTransform &transform_out);
#endif
#ifdef USE_FCL
    void setTransform(const KDL::Frame &fkPose, const KDL::Frame& fixPose, fcl::Transform3f &transform_out);
#endif

    /**
     * @brief Calculate the closest distance between two collision bodies
     * @param Input: The two collision bodies and their poses, output: The closest points with the corresponding distance en normal vector between them
     */
#ifdef USE_BULLET
    void distanceCalculation(btConvexShape &shapeA, btConvexShape &shapeB, btTransform& transformA, btTransform& transformB, btPointCollector& distance_out);
#endif
#ifdef USE_FCL
    static boost::shared_ptr<fcl::CollisionGeometry> shapeToMesh(const fcl::CollisionGeometry &shape, const fcl::Transform3f pose = fcl::Transform3f());
    void distanceCalculation(const fcl::CollisionObject* o1, const fcl::CollisionObject* o2, fcl::DistanceResult& result);
#endif

    /**
     * @brief Select the minimal closest distance
     * @param Vector with all closest distances from a collision body, Vector with the minimal closest distances
     */
#ifdef USE_BULLET
    void pickMinimumDistance(std::vector<Distance> &calculatedDistances, std::vector<Distance> &minimumDistances);
#endif
#ifdef USE_FCL
    void pickMinimumDistance(std::vector<Distance2> &calculatedDistances, std::vector<Distance2> &minimumDistances);
#endif
    /**
     * @brief Calculate the amplitude of the repulsive forces
     * @param Vector with the minimal closest distances, Vector with the repulsive forces
     */
    // ToDo: use membervariablefor collisionAvoidanceParameters
    // ToDo: use only 1 implementation for calculateRepulsiveForce
    void calculateRepulsiveForce(      std::vector<Distance>  &minimumDistances, std::vector<RepulsiveForce>  &repulsiveForces, collisionAvoidanceParameters::Parameters &param);
    void calculateRepulsiveForce(const std::vector<Distance>  &minimumDistances, std::vector<RepulsiveForce2> &repulsiveForces, const collisionAvoidanceParameters::Parameters &param);
    void calculateRepulsiveForce(const std::vector<Distance2> &minimumDistances, std::vector<RepulsiveForce2> &repulsiveForces, const collisionAvoidanceParameters::Parameters &param);

    /**
     * @brief Calculate the wrenches as a function of the repulsive forces
     * @param Input: Vector with the repulsive forces, Output: Vector with the wrenches
     */
    void calculateWrenches(      std::vector<RepulsiveForce>  &repulsive_forces);
    void calculateWrenches(const std::vector<RepulsiveForce2> &repulsive_forces);

    /**
     * @brief Output the wrench to the KDL tree
     * @param Vector with wrenches
     */
    void outputWrenches(std::vector<Wrench> &wrenches);

    /**
     * @brief Visualize the collision avoidance in RVIZ
     * @param Vector with minimal distances
     */
    void visualize(std::vector<Distance> &min_distances) const;
    void visualizeRepulsiveForces(std::vector<Distance2> &min_distances) const;

    /**
     * @brief Construct the visualization markers to visualize the collision model in RVIZ
     * @param Collision body information
     */
    void visualizeCollisionModel(RobotState::CollisionBody collisionBody,int id)  const;
    void visualizeCollisionModelFCL(RobotState::CollisionBody collisionBody,int id)  const;

    /**
     * @brief Construct the visualization markers to visualize the repulsive forces in RVIZ
     * @param Collision vector
     */
    void visualizeRepulsiveForce(Distance &d_min, int id) const;
    void visualizeRepulsiveForce(Distance2 &d_min,int id) const;

    /**
     * @brief Construct the visualization markers to visualize the bounding box in RVIZ
     * @param Minimum and maximum point of the bounding box
     */
    void visualizeBBX(octomath::Vector3 min, octomath::Vector3 max, int id) const;

    /**
     * @brief Find the outer points of the collision models for the bounding box construction in /map frame
     * @param Input: The collision body, Output: The minimum and maximum point of the collision body in /map frame
     */
    void findOuterPoints(RobotState::CollisionBody& collisionBody, btVector3 &min, btVector3 &max);


};

#endif
