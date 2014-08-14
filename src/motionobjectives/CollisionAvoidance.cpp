#include "amigo_whole_body_controller/motionobjectives/CollisionAvoidance.h"
#include <math.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#ifdef USE_FCL

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <geolib_fcl/tools.h>

#endif

// debug information for transformations
//#define VERBOSE_TRANSFORMS

// tsv formatted repulsive forces
//#define DEBUG_REPULSIVE_FORCE

// amount of fcl mesh nodes generated
#define GEOM_CYLINDER_tot   8
#define GEOM_CYLINDER_h_num 8
#define GEOM_CONE_tot       8
#define GEOM_CONE_h_num     8
#define GEOM_SPHERE_seg     8
#define GEOM_SPHERE_ring    8



namespace wbc {

/// @brief Distance data stores the distance request and the result given by distance algorithm.
struct DistanceData
{
  DistanceData()
  {
    done = false;
    verbose = false;
  }

  /// @brief Distance request
  fcl::DistanceRequest request;

  /// @brief Distance result
  fcl::DistanceResult result;

  /// @brief Whether the distance iteration can stop
  bool done;

  bool verbose;

};

CollisionAvoidance::CollisionAvoidance(collisionAvoidanceParameters &parameters, const double Ts)
    : ca_param_(parameters), Ts_ (Ts), octomap_(NULL)
{
    /// Status is always 2 (always active)
    type_     = "CollisionAvoidance";
    status_   = 2;
    priority_ = 1;
    cost_     = 0.0;
}

CollisionAvoidance::~CollisionAvoidance()
{
    delete octomap_;
    delete depthSolver;
    delete simplexSolver;
}

bool CollisionAvoidance::initialize(RobotState &robotstate)
{
    robot_state_ = &robotstate;

    ROS_INFO_STREAM("Initializing Obstacle Avoidance, ");

    // Get node handle
    ros::NodeHandle n("~");
    std::string ns = ros::this_node::getName();

    // Initialize output topics
    pub_model_marker_      = n.advertise<visualization_msgs::MarkerArray>("/whole_body_controller/collision_model_markers/", 10);
    pub_model_marker_fcl_  = n.advertise<visualization_msgs::Marker>("/whole_body_controller/collision_model_markers_fcl/", 10);
    pub_forces_marker_     = n.advertise<visualization_msgs::MarkerArray>("/whole_body_controller/repulsive_forces_markers/", 10);
    pub_forces_marker_fcl_ = n.advertise<visualization_msgs::MarkerArray>("/whole_body_controller/repulsive_forces_markers_fcl/", 10);
    pub_bbx_marker_        = n.advertise<visualization_msgs::MarkerArray>("/whole_body_controller/bbx_markers/", 10);

    // Initialize OctoMap
    octomap_ = new octomap::OcTreeStamped(ca_param_.environment_collision.octomap_resolution);

    // Initialize a frame that indicates no fix is required from a FK pose to the collision shape
    no_fix_.Identity();

    initializeCollisionModel(robotstate);

    // Initialize solver for distance calculation
    depthSolver = new btMinkowskiPenetrationDepthSolver;
    simplexSolver = new btVoronoiSimplexSolver;

    /// Initialize vector and matrix objects
    unsigned int number_joints = robot_state_->getNrJoints();
    jacobian_.resize(0,number_joints);
    jacobian_.setZero();
    jacobian_pre_alloc_.resize(100,number_joints);//ToDo: can't we do this any nicer?
    jacobian_pre_alloc_.setZero();
    wrenches_pre_alloc_.resize(100);//ToDo: can't we do this any nicer?
    wrenches_pre_alloc_.setZero();
    torques_.resize(number_joints);
    torques_.setZero();

    ROS_INFO_STREAM("Initialized Obstacle Avoidance");

    client_.startThread();

    statsPublisher_.initialize();

    return true;
}

void CollisionAvoidance::apply(RobotState &robotstate)
{
    statsPublisher_.startTimer("CollisionAvoidance::apply");

    /// Reset stuff
    jacobian_pre_alloc_.setZero();
    wrenches_pre_alloc_.setZero();
    torques_.setZero();
    cost_ = 0.0;

    robot_state_ = &robotstate; // TODO: is this really necessary? This already happens in the contructor
    calculateTransform();

    // Calculate the wrenches as a result of (self-)collision avoidance
    std::vector<Distance> min_distances_total;
    std::vector<Distance2> min_distances_total_fcl;
    std::vector<Distance2> min_distances_total_fast;
    std::vector<RepulsiveForce> repulsive_forces_total;  // this will get removed eventually
    std::vector<RepulsiveForce> repulsive_forces_total_fcl;

    statsPublisher_.startTimer("CollisionAvoidance::selfCollision");

    // Calculate the repulsive forces as a result of the self-collision avoidance.
    selfCollision(min_distances_total, min_distances_total_fcl);

    statsPublisher_.stopTimer("CollisionAvoidance::selfCollision");

    statsPublisher_.startTimer("CollisionAvoidance::selfCollisionFast");
    selfCollisionFast(min_distances_total_fast);
    statsPublisher_.stopTimer("CollisionAvoidance::selfCollisionFast");

    // Calculate the repulsive forces as a result of the environment collision avoidance.
    /*
    if (octomap_){
        if (octomap_->size() > 0)
        {
            environmentCollision(min_distances_total);
        }
        else{
            ROS_WARN_ONCE("Collision Avoidance: No octomap created!");
        }
    }
    */

#ifdef USE_FCL
    statsPublisher_.startTimer("CollisionAvoidance::environmentCollisionVWM");

    // Calculate the repulsive forces as a result of the volumetric world model
    environmentCollisionVWM(min_distances_total_fcl);

    statsPublisher_.stopTimer("CollisionAvoidance::environmentCollisionVWM");
#endif

    statsPublisher_.startTimer("CollisionAvoidance::calculateRepulsiveForce");

    /// Calculate the repulsive forces and the corresponding 'wrenches' and Jacobians from the minimum distances
    calculateRepulsiveForce(min_distances_total,     repulsive_forces_total,     ca_param_.self_collision);
#ifdef USE_FCL
    calculateRepulsiveForce(min_distances_total_fcl, repulsive_forces_total_fcl, ca_param_.self_collision);
#endif

    statsPublisher_.stopTimer("CollisionAvoidance::calculateRepulsiveForce");

#ifdef DEBUG_REPULSIVE_FORCE
    uint64_t t = ros::Time::now().toNSec() / 1000;
    for(std::vector<RepulsiveForce>::iterator it = repulsive_forces_total.begin(); it < repulsive_forces_total.end(); it++) {
        std::cout << "rp_bt "   << t << " " << *it << std::endl;
    }
    for(std::vector<RepulsiveForce>::iterator it = repulsive_forces_total_fcl.begin(); it < repulsive_forces_total_fcl.end(); it++) {
        std::cout << "rp_fcl "  << t << " " << *it << std::endl;
    }
#endif

    statsPublisher_.startTimer("CollisionAvoidance::calculateWrenches");

    calculateWrenches(repulsive_forces_total_fcl);

    statsPublisher_.stopTimer("CollisionAvoidance::calculateWrenches");


    /// Output
    statsPublisher_.startTimer("CollisionAvoidance::visualize");

    visualize(min_distances_total);
    visualizeRepulsiveForces(min_distances_total_fcl);

    statsPublisher_.stopTimer("CollisionAvoidance::visualize");

    statsPublisher_.stopTimer("CollisionAvoidance::apply");
    statsPublisher_.publish();
}

void CollisionAvoidance::addObjectCollisionModel(const std::string& frame_id) {

    /// Dimensions: // ToDo: make variable;
    double x_dim = 0.05;
    double y_dim = 0.05;
    double z_dim = 0.10;
    double r     = 0.05/2;

    /// Initialize collision body
    RobotState::CollisionBody object_collision_body;
    object_collision_body.name_collision_body = "object";           //ToDo: possibly add WM ID?
    object_collision_body.collision_shape.shape_type = "CylinderZ"; //ToDo: make this variable
    object_collision_body.collision_shape.dimensions.x = x_dim;     //ToDo: make this variable
    object_collision_body.collision_shape.dimensions.y = y_dim;     //ToDo: make this variable
    object_collision_body.collision_shape.dimensions.z = z_dim;     //ToDo: make this variable
    object_collision_body.frame_id = frame_id;

#ifdef USE_BULLET
    object_collision_body.bt_shape = new btCylinderShapeZ(btVector3(x_dim, y_dim, z_dim));
#endif
#ifdef USE_FCL
    object_collision_body.fcl_shape = shapeToMesh(fcl::Cylinder(r, z_dim));
    object_collision_body.fcl_object = boost::shared_ptr<fcl::CollisionObject>(new fcl::CollisionObject(object_collision_body.fcl_shape));
#endif

    object_collision_body.fix_pose.Identity();                      //ToDo: is now initialized as p = [0,0,0], M = [0,0,0,1]

    /// Add collision body to correct frame/group/etc.
    // Robotstate pointer is member variable
    // ToDo

    /// Add to exclusions
    // ToDo

}

void CollisionAvoidance::removeObjectCollisionModel() {

    /// Remove object collision model from frame/group/etc.
    // ToDo

    /// Remove from exclusions
    // ToDo

}

void CollisionAvoidance::selfCollision(std::vector<Distance> &min_distances, std::vector<Distance2> &min_distances2)
{
    // Loop through all collision groups
    for (std::vector< std::vector<RobotState::CollisionBody> >::iterator itrGroup = robot_state_->robot_.groups.begin(); itrGroup != robot_state_->robot_.groups.end(); ++itrGroup)
    {        
        std::vector<RobotState::CollisionBody> &Group = *itrGroup;
        for (std::vector<RobotState::CollisionBody>::iterator itrBody = Group.begin(); itrBody != Group.end(); ++itrBody)
        {
            // Loop through al the bodies of the group
            std::vector<Distance> distanceCollection;
            std::vector<Distance2> distanceCollection2;
            RobotState::CollisionBody &currentBody = *itrBody;

            // Distance check with other groups if the name of the groups is different than the current group
            for (std::vector< std::vector<RobotState::CollisionBody> >::iterator itrCollisionGroup = robot_state_->robot_.groups.begin(); itrCollisionGroup != robot_state_->robot_.groups.end(); ++itrCollisionGroup)
            {
                std::vector<RobotState::CollisionBody> &collisionGroup = *itrCollisionGroup;
                if (collisionGroup.front().name_collision_body != Group.front().name_collision_body)
                {
                    // Loop trough al the bodies of the group
                    for (std::vector<RobotState::CollisionBody>::iterator itrCollisionBody = collisionGroup.begin(); itrCollisionBody != collisionGroup.end(); ++itrCollisionBody)
                    {
                        RobotState::CollisionBody &collisionBody = *itrCollisionBody;

                        // Check for exclusions
                        bool skip_check = false;
                        for (std::vector<RobotState::Exclusion> ::iterator itrExcl = robot_state_->exclusion_checks.checks.begin(); itrExcl != robot_state_->exclusion_checks.checks.end(); ++itrExcl)
                        {
                            RobotState::Exclusion &excluded_bodies = *itrExcl;
                            if ( (currentBody.name_collision_body == excluded_bodies.name_body_A && collisionBody.name_collision_body == excluded_bodies.name_body_B) ||
                                 (collisionBody.name_collision_body == excluded_bodies.name_body_A && currentBody.name_collision_body == excluded_bodies.name_body_B) )
                            {
                                skip_check = true;
                            }
                        }

                        if (skip_check == false)
                        {
#ifdef USE_BULLET
                            Distance distance;
                            distance.frame_id = currentBody.frame_id;
                            distanceCalculation(*currentBody.bt_shape, *collisionBody.bt_shape, currentBody.bt_transform, collisionBody.bt_transform, distance.bt_distance);
                            distanceCollection.push_back(distance);
#endif
#ifdef USE_FCL
                            Distance2 distance2;
                            distance2.frame_id = currentBody.frame_id;
                            distanceCalculation(currentBody.fcl_object.get(), collisionBody.fcl_object.get(), distance2.result);
                            distanceCollection2.push_back(distance2);
#endif
                        }
                    }
                }
            }
            // Find minimum distance
#ifdef USE_BULLET
            pickMinimumDistance(distanceCollection,  min_distances);
#endif
#ifdef USE_FCL
            pickMinimumDistance(distanceCollection2, min_distances2);
#endif
        }
    }
}

#ifdef USE_FCL
bool selfCollisionDistanceFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_, fcl::FCL_REAL& dist)
{
  DistanceData* cdata = static_cast<DistanceData*>(cdata_);

  const fcl::DistanceRequest& request = cdata->request;
  fcl::DistanceResult& result = cdata->result;

  const CollisionGeometryData* cd1 = static_cast<const CollisionGeometryData*>(o1->getCollisionGeometry()->getUserData());
  const CollisionGeometryData* cd2 = static_cast<const CollisionGeometryData*>(o2->getCollisionGeometry()->getUserData());

  if(cdata->done) { dist = result.min_distance; return true; }

  fcl::distance(o1, o2, request, result);

  dist = result.min_distance;

  if(dist <= 0) return true; // in collision or in touch

  return cdata->done;
}

void CollisionAvoidance::selfCollisionFast(std::vector<Distance2> &min_distances)
{
    DistanceData cdata;
    cdata.verbose = true;
    cdata.request.enable_nearest_points = true;

    std::cout << "starting with self collision" << std::endl;

    selfCollisionManager.distance(&cdata, selfCollisionDistanceFunction);

    fcl::DistanceResult result = cdata.result;
}
#endif

void CollisionAvoidance::environmentCollision(std::vector<Distance> &min_distances)
{
    double xmin_octomap,ymin_octomap,zmin_octomap;
    double xmax_octomap,ymax_octomap,zmax_octomap;
    octomap_->getMetricMin(xmin_octomap,ymin_octomap,zmin_octomap);
    octomap_->getMetricMax(xmax_octomap,ymax_octomap,zmax_octomap);
    min_.clear();
    max_.clear();

    // Calculate closest distances using Bullet
    for (std::vector< std::vector<RobotState::CollisionBody> >::iterator itrGroups = robot_state_->robot_.groups.begin(); itrGroups != robot_state_->robot_.groups.end(); ++itrGroups)
    {
        std::vector<RobotState::CollisionBody> &group = *itrGroups;
        for (std::vector<RobotState::CollisionBody>::iterator itrBodies = group.begin(); itrBodies != group.end(); ++itrBodies)
        {
            RobotState::CollisionBody &collisionBody = *itrBodies;

            // Find the Voxels that are in range of collision body
            double xmin_bbx,ymin_bbx,zmin_bbx;
            double xmax_bbx,ymax_bbx,zmax_bbx;
            Distance distance;
            //btVector3 bbx_dim;
            btVector3 min_cb = collisionBody.bt_transform.getOrigin();
            btVector3 max_cb = collisionBody.bt_transform.getOrigin();

            findOuterPoints(collisionBody, min_cb, max_cb);

            // Add the threshold distance to the found axis aligned BBX
            xmin_bbx = min_cb[0]-ca_param_.environment_collision.d_threshold;
            ymin_bbx = min_cb[1]-ca_param_.environment_collision.d_threshold;
            zmin_bbx = min_cb[2]-ca_param_.environment_collision.d_threshold;

            xmax_bbx = max_cb[0]+ca_param_.environment_collision.d_threshold;
            ymax_bbx = max_cb[1]+ca_param_.environment_collision.d_threshold;
            zmax_bbx = max_cb[2]+ca_param_.environment_collision.d_threshold;

            // ToDo:: Get rid of hardcoded bug fix
            if (collisionBody.name_collision_body == "BaseTop")
            {
                zmin_bbx = zmin_bbx - 0.15;
            }

            // Check whether the BBX is inside the OctoMap, otherwise take OctoMap dimensions
            if (xmin_octomap > xmin_bbx) {
                xmin_bbx = xmin_octomap;
            }
            if (ymin_octomap > ymin_bbx) {
                ymin_bbx = ymin_octomap;
            }
            if (zmin_octomap > zmin_bbx) {
                zmin_bbx = xmin_octomap;
            }

            if (xmax_octomap < xmax_bbx) {
                xmax_bbx = xmax_octomap;
            }
            if (ymax_octomap < ymax_bbx) {
                ymax_bbx = ymax_octomap;
            }
            if (zmax_octomap < zmax_bbx) {
                zmax_bbx = zmax_octomap;
            }


            // Set bounding box using its minimum and maximum coordinate
            octomath::Vector3 min = octomath::Vector3(xmin_bbx,
                                                      ymin_bbx,
                                                      zmin_bbx);

            octomath::Vector3 max = octomath::Vector3(xmax_bbx,
                                                      ymax_bbx,
                                                      zmax_bbx);


            // Store bbx min max in vector for visualization
            min_.push_back(min);
            max_.push_back(max);


            // Find and store all occupied voxels
            std::vector<Voxel> in_range_voxels;
            for(octomap::OcTreeStamped::leaf_bbx_iterator it = octomap_->begin_leafs_bbx(min,max), end=octomap_->end_leafs_bbx(); it!= end; ++it)
            {
                Voxel vox;
                if (octomap_->isNodeOccupied(*it))
                {
                    // Convert voxel center point to KDL frame
                    vox.center_point.p.x(it.getX());
                    vox.center_point.p.y(it.getY());
                    vox.center_point.p.z(it.getZ());
                    vox.center_point.M.Quaternion(0,0,0,1);

                    // Get Voxel size
                    vox.size_voxel = octomap_->getNodeSize(it.getDepth());

                    in_range_voxels.push_back(vox);
                }
            }

            std::vector<Distance> distanceCollection;
            for (std::vector<Voxel>::iterator itrVox = in_range_voxels.begin(); itrVox != in_range_voxels.end(); ++itrVox)
            {
                Voxel vox = *itrVox;
                RobotState::CollisionBody envBody;

                // Construct a Bullet Convex Shape of every Voxel within range
                envBody.bt_shape = new btBoxShape(btVector3(0.5*vox.size_voxel,0.5*vox.size_voxel,0.5*vox.size_voxel));
                setTransform(vox.center_point, no_fix_, envBody.bt_transform);

                distance.frame_id = collisionBody.frame_id;
                distanceCalculation(*collisionBody.bt_shape,*envBody.bt_shape,collisionBody.bt_transform,envBody.bt_transform,distance.bt_distance);

                distanceCollection.push_back(distance);
                delete envBody.bt_shape;
            }
            // Find minimum distance
            pickMinimumDistance(distanceCollection,min_distances);
        }
    }
}

#ifdef USE_FCL
bool environmentCollisionDistanceFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_, fcl::FCL_REAL& dist)
{
  DistanceData* cdata = static_cast<DistanceData*>(cdata_);
  const fcl::DistanceRequest& request = cdata->request;
  fcl::DistanceResult& result = cdata->result;

  if(cdata->done) { dist = result.min_distance; return true; }

  fcl::distance(o1, o2, request, result);

  dist = result.min_distance;

  if(dist <= 0) return true; // in collision or in touch

  return cdata->done;
}

void CollisionAvoidance::environmentCollisionVWM(std::vector<Distance2> &min_distances) {

    std::vector< boost::shared_ptr<fcl::CollisionObject> > objects = client_.getWorldObjects();
    if (!objects.size()) {
        return;
    }

    fcl::DynamicAABBTreeCollisionManager manager;
    std::vector< boost::shared_ptr<fcl::CollisionObject> >::iterator it;
    for (it = objects.begin(); it < objects.end(); it++) {
        boost::shared_ptr<fcl::CollisionObject> obj = *it;
        manager.registerObject(obj.get());
    }
    manager.setup();

    // Calculate closest distances using Bullet
    for (std::vector< std::vector<RobotState::CollisionBody> >::iterator itrGroups = robot_state_->robot_.groups.begin(); itrGroups != robot_state_->robot_.groups.end(); ++itrGroups)
    {
        std::vector<RobotState::CollisionBody> &group = *itrGroups;
        for (std::vector<RobotState::CollisionBody>::iterator itrBodies = group.begin(); itrBodies != group.end(); ++itrBodies)
        {
            RobotState::CollisionBody &collisionBody = *itrBodies;

            DistanceData cdata;
            cdata.request.enable_nearest_points = true;
            manager.distance(collisionBody.fcl_object.get(), &cdata, environmentCollisionDistanceFunction);

            Distance2 distance;
            distance.frame_id = collisionBody.frame_id;
            distance.result = cdata.result;
            min_distances.push_back(distance);
        }
    }
    manager.clear();
}
#endif

void CollisionAvoidance::calculateWrenches(const std::vector<RepulsiveForce> &repulsive_forces)
{
    //std::cout << "CA calculate wrenches" << std::endl;
    unsigned int row_index = 0;
    for (std::vector<RepulsiveForce>::const_iterator itrRF = repulsive_forces.begin(); itrRF != repulsive_forces.end(); ++itrRF)
    {
        const RepulsiveForce &RF = *itrRF;
        /*
        // Testcase: replace the for-loop above and possibly comment the 'Calculate delta p in map coordinates' section (depends on RF.pointOnA)
    for (unsigned int cn = 0; cn<1; cn++) {
        RepulsiveForce RF;
        RF.frame_id  = "grippoint_right";
        RF.amplitude = 1.0;
        RF.direction = btVector3(0.0, 0.0, 1.0);
        RF.pointOnA  = btVector3(0.43, -0.22, 0.9615);
        //double dpx = 0.0; double dpy =  0.0; double dpz = 0.0;
     // End Testcase
        */
        KDL::Frame p0;
        Eigen::VectorXd wrench_calc(6);
        wrench_calc.setZero();


        /// Find the frame of the kinematic model on which the wrench acts.
        for (std::vector< std::vector<RobotState::CollisionBody> >::iterator itrGroups = robot_state_->robot_.groups.begin(); itrGroups != robot_state_->robot_.groups.end(); ++itrGroups)
        {
            std::vector<RobotState::CollisionBody> &group = *itrGroups;
            for (std::vector<RobotState::CollisionBody>::iterator itrBodies = group.begin(); itrBodies != group.end(); ++itrBodies)
            {
                RobotState::CollisionBody &collisionBody = *itrBodies;
                if (collisionBody.frame_id == RF.frame_id )
                {
                    p0 = collisionBody.fk_pose;
                }
            }
        }

        /// Calculate delta p in map coordinates
        double dpx = RF.pointOnA[0] - p0.p.x();
        double dpy = RF.pointOnA[1] - p0.p.y();
        double dpz = RF.pointOnA[2] - p0.p.z();

        /// Compute 6xn Jacobian (this is w.r.t.root frame of the kinematic tree, so in this case: base_link)
        Eigen::MatrixXd temp_jacobian(6,robot_state_->getNrJoints());// ToDo: get rid of this
        KDL::Jacobian partial_jacobian(robot_state_->getNrJoints());
        robot_state_->tree_.calcPartialJacobian(RF.frame_id, temp_jacobian);
        partial_jacobian.data = temp_jacobian;

        /// Change reference point: the force does not always act at the end of a certain link
        partial_jacobian.changeRefPoint(KDL::Vector(dpx, dpy, dpz));

        /// Change base: the Jacobian is computed w.r.t. base_link instead of map, while the force is expressed in map
        std::map<std::string, KDL::Frame>::iterator itrFK = robot_state_->fk_poses_.find("base_link"); //ToDo: don't hardcode???
        KDL::Frame BaseFrame_in_map = itrFK->second;
        partial_jacobian.changeBase(BaseFrame_in_map.M);

        /// Premultiply first three rows with force direction to get one row
        //std::cout << "Force on " << RF.frame_id << " in direction " << RF.direction.getX() << ", " << RF.direction.getY() << ", " << RF.direction.getZ() << std::endl;
        //ROS_INFO("Multiplying [%i, %i] x [%i, %i] into [%i,%i]", force_direction.rows(), force_direction.cols(),
        //         partial_jacobian.data.block(0,0,3,robot_state_->getNrJoints()).rows(), partial_jacobian.data.block(0,0,3,robot_state_->getNrJoints()).cols(),
        //         jacobian_pre_alloc_.block(row_index, 0, 1, robot_state_->getNrJoints()).rows(), jacobian_pre_alloc_.block(row_index, 0, 1, robot_state_->getNrJoints()).cols());
        jacobian_pre_alloc_.block(row_index, 0, 1, robot_state_->getNrJoints()) =  RF.direction.transpose() * partial_jacobian.data.block(0, 0, 3, robot_state_->getNrJoints());

        /// Add force magnitude to list
        wrenches_pre_alloc_(row_index) = RF.amplitude;

        /// Add amplitude to cost
        cost_ +=RF.amplitude;

        row_index++;

    }

    /// Extract total Jacobian and vector with wrenches
    jacobian_ = jacobian_pre_alloc_.block(0, 0, row_index, robot_state_->getNrJoints());
    Eigen::VectorXd wrenches = wrenches_pre_alloc_.block(0, 0, row_index, 1);
    //std::cout << "Wrenches: \n" << wrenches_new_ << std::endl;

    /// Multiply to get torques
    torques_ = jacobian_.transpose() * wrenches;
    //std::cout << "CA Torques: \n" << torques_ << std::endl;
}

void CollisionAvoidance::visualize(std::vector<Distance> &min_distances) const
{
    int id = 0;
    // Loop through the collision bodies and visualize them.
    for (std::vector< std::vector<RobotState::CollisionBody> >::const_iterator itrGroups = robot_state_->robot_.groups.begin(); itrGroups != robot_state_->robot_.groups.end(); ++itrGroups)
    {
        std::vector<RobotState::CollisionBody> group = *itrGroups;
        for (std::vector<RobotState::CollisionBody>::iterator itrsBodies = group.begin(); itrsBodies != group.end(); ++itrsBodies)
        {
            RobotState::CollisionBody &collisionBody = *itrsBodies;
#ifdef USE_BULLET
            visualizeCollisionModel(collisionBody,id++);
#endif
#ifdef USE_FCL
            visualizeCollisionModelFCL(collisionBody,id++);
#endif
        }
    }

    // Loop through the closest distance vectors and visualize them.
    for (std::vector<Distance>::const_iterator itrdmin = min_distances.begin(); itrdmin != min_distances.end(); ++itrdmin)
    {
        Distance dmin = *itrdmin;
        visualizeRepulsiveForce(dmin,id++);
    }

    for (unsigned int i = 0; i < min_.size(); i++)
    {
        visualizeBBX(min_[i], max_[i], id++);
    }
}

void CollisionAvoidance::visualizeRepulsiveForces(std::vector<Distance2> &min_distances) const
{
    int id = 0;
    // Loop through the closest distance vectors and visualize them.
    for (std::vector<Distance2>::const_iterator itrdmin = min_distances.begin(); itrdmin != min_distances.end(); ++itrdmin)
    {
        Distance2 dmin = *itrdmin;
        visualizeRepulsiveForce(dmin,id++);
    }
}


void CollisionAvoidance::initializeCollisionModel(RobotState& robotstate)
{
    for (std::vector< std::vector<RobotState::CollisionBody> >::iterator itrGroups = robotstate.robot_.groups.begin(); itrGroups != robotstate.robot_.groups.end(); ++itrGroups)
    {
        std::vector<RobotState::CollisionBody> &group = *itrGroups;
        for (std::vector<RobotState::CollisionBody>::iterator itrBodies = group.begin(); itrBodies != group.end(); ++itrBodies)
        {
            RobotState::CollisionBody &collisionBody = *itrBodies;
            std::string type = collisionBody.collision_shape.shape_type;
            double x = collisionBody.collision_shape.dimensions.x;
            double y = collisionBody.collision_shape.dimensions.y;
            double z = collisionBody.collision_shape.dimensions.z;

            std::cout << "initializeCollisionModel " << collisionBody.name_collision_body << " of type " << type << std::endl;
            std::cout << "\tx: " << x << std::endl;
            std::cout << "\ty: " << y << std::endl;
            std::cout << "\tz: " << z << std::endl;

            if (type=="Box")
            {
#ifdef USE_BULLET
                collisionBody.bt_shape = new btBoxShape(btVector3(x,y,z));
#endif
#ifdef USE_FCL
                collisionBody.fcl_shape = shapeToMesh(fcl::Box(x*2, y*2, z*2));
#endif
            }
            else if (type == "Sphere")
            {
#ifdef USE_BULLET
                collisionBody.bt_shape = new btSphereShape(x);
#endif
#ifdef USE_FCL
                collisionBody.fcl_shape = shapeToMesh(fcl::Sphere(x));
#endif
            }
            else if (type == "Cone")
            {
#ifdef USE_BULLET
                collisionBody.bt_shape = new btConeShapeZ(x-0.05,2*z);
#endif
#ifdef USE_FCL
                collisionBody.fcl_shape = shapeToMesh(fcl::Cone(x, 2*z));
                assert(x == y);
#endif
            }
            else if (type == "CylinderY")
            {
#ifdef USE_BULLET
                collisionBody.bt_shape = new btCylinderShape(btVector3(x,y,z));
#endif
#ifdef USE_FCL
                assert(x == z);
                // fcl cylinders are oriented around the z axis, so we must rotate pi/2 around x
                fcl::Quaternion3f q;
                q.fromAxisAngle(fcl::Vec3f(1, 0, 0), M_PI_2);
                collisionBody.fcl_shape = shapeToMesh(fcl::Cylinder(x, y*2), fcl::Transform3f(q));
#endif
            }
            else if (type == "CylinderZ")
            {
#ifdef USE_BULLET
                collisionBody.bt_shape = new btCylinderShapeZ(btVector3(x,y,z));
#endif
#ifdef USE_FCL
                assert(x == y);
                collisionBody.fcl_shape = shapeToMesh(fcl::Cylinder(x, z*2)); // TODO: check what happens with z
#endif
            }
            else
            {
                ROS_WARN("Collision shape '%s' not found", type.c_str());
            }
#ifdef USE_FCL
            collisionBody.fcl_object = boost::shared_ptr<fcl::CollisionObject>(new fcl::CollisionObject(collisionBody.fcl_shape));

            selfCollisionManager.registerObject(collisionBody.fcl_object.get());
#endif
        }
    }

    selfCollisionManager.setup();
}


void CollisionAvoidance::calculateTransform()
{
    // Loop throught the collision bodies and calculate the transform from /map frame to the collision bodies center frame.
    for (std::vector< std::vector<RobotState::CollisionBody> >::iterator it = robot_state_->robot_.groups.begin(); it != robot_state_->robot_.groups.end(); ++it)
    {
        std::vector<RobotState::CollisionBody> &group = *it;

        for (std::vector<RobotState::CollisionBody>::iterator it = group.begin(); it != group.end(); ++it)
        {
            RobotState::CollisionBody &collisionBody = *it;
#ifdef VERBOSE_TRANSFORMS
            ROS_INFO("transformation of %s", collisionBody.frame_id.c_str());
#endif

#ifdef USE_BULLET
            setTransform(collisionBody.fk_pose, collisionBody.fix_pose, collisionBody.bt_transform);
#endif
#ifdef USE_FCL
            setTransform(collisionBody.fk_pose, collisionBody.fix_pose, collisionBody.fcl_transform);
            collisionBody.fcl_object.get()->setTransform(collisionBody.fcl_transform);
#endif

#ifdef VERBOSE_TRANSFORMS
            btVector3 bt_t = collisionBody.bt_transform.getOrigin();
            fcl::Vec3f fcl_t = collisionBody.fcl_transform.getTranslation();
            ROS_INFO("\tbt_translation (%lf,%lf,%lf)", bt_t.getX(), bt_t.getY(), bt_t.getZ());
            ROS_INFO("\tbt_translation (%lf,%lf,%lf)", fcl_t[0], fcl_t[1], fcl_t[2]);
#endif
        }
    }
}


#ifdef USE_BULLET
void CollisionAvoidance::setTransform(const KDL::Frame &fkPose, const KDL::Frame &fixPose,btTransform &transform_out)
{
    btTransform fkTransform;
    btTransform fixTransform;
    double x,y,z,w;

    fkTransform.setOrigin(btVector3(fkPose.p.x(),
                                    fkPose.p.y(),
                                    fkPose.p.z()));

    fkPose.M.GetQuaternion(x,y,z,w);
    fkTransform.setRotation(btQuaternion(x,y,z,w));


    fixTransform.setOrigin(btVector3(fixPose.p.x(),
                                     fixPose.p.y(),
                                     fixPose.p.z()));

    fixPose.M.GetQuaternion(x,y,z,w);
    fixTransform.setRotation(btQuaternion(x,y,z,w));

    transform_out.mult(fkTransform,fixTransform);
}
#endif
#ifdef USE_FCL
void CollisionAvoidance::setTransform(const KDL::Frame &fkPose, const KDL::Frame &fixPose, fcl::Transform3f &transform_out)
{
    fcl::Transform3f fkTransform;
    fcl::Transform3f fixTransform;
    double x,y,z,w;

    fkTransform.setTranslation(fcl::Vec3f(fkPose.p.x(),
                                          fkPose.p.y(),
                                          fkPose.p.z()));

    fkPose.M.GetQuaternion(x,y,z,w);
    fkTransform.setQuatRotation(fcl::Quaternion3f(w,x,y,z));


    fixTransform.setTranslation(fcl::Vec3f(fixPose.p.x(),
                                           fixPose.p.y(),
                                           fixPose.p.z()));

    fixPose.M.GetQuaternion(x,y,z,w);
    fixTransform.setQuatRotation(fcl::Quaternion3f(w,x,y,z));

    transform_out = fkTransform*fixTransform;
}
#endif


#ifdef USE_BULLET
void CollisionAvoidance::distanceCalculation(btConvexShape& shapeA, btConvexShape& shapeB, btTransform& transformA, btTransform& transformB, btPointCollector &distance_out)
{
    btGjkPairDetector convexConvex(&shapeA, &shapeB, simplexSolver, depthSolver);

    // Set input transforms
    btGjkPairDetector::ClosestPointInput input;
    input.m_transformA = transformA;
    input.m_transformB = transformB;

    // Calculate closest distance
    convexConvex.getClosestPoints(input, distance_out, 0);
}
#endif

#ifdef USE_FCL
boost::shared_ptr<fcl::CollisionGeometry> CollisionAvoidance::shapeToMesh(
        const fcl::CollisionGeometry &shape,
        const fcl::Transform3f pose /* defaults to the null transform */)
{
    fcl::BVHModel<fcl::OBBRSS>* model = new fcl::BVHModel<fcl::OBBRSS>();

    fcl::NODE_TYPE shapeType = shape.getNodeType();
    switch(shapeType)
    {
    case fcl::GEOM_BOX: {
        const fcl::Box& geom = dynamic_cast<const fcl::Box&>(shape);
        fcl::generateBVHModel(*model, geom, pose);
        break;
    }
    case fcl::GEOM_CYLINDER: {
        const fcl::Cylinder& geom = dynamic_cast<const fcl::Cylinder&>(shape);
        fcl::generateBVHModel(*model, geom, pose, GEOM_CYLINDER_tot, GEOM_CYLINDER_h_num);
        break;
    }
    case fcl::GEOM_CONE: {
        const fcl::Cone& geom = dynamic_cast<const fcl::Cone&>(shape);
        fcl::generateBVHModel(*model, geom, pose, GEOM_CONE_tot, GEOM_CONE_h_num);
        break;
    }
    case fcl::GEOM_SPHERE: {
        const fcl::Sphere& geom = dynamic_cast<const fcl::Sphere&>(shape);
        fcl::generateBVHModel(*model, geom, pose, GEOM_SPHERE_seg, GEOM_SPHERE_ring);
        break;
    }
    default:
    {
        ROS_WARN_ONCE("error converting unknown fcl shape: %i", shapeType);
        break;
    }
    }

    return boost::shared_ptr<fcl::CollisionGeometry>(model);
}

void CollisionAvoidance::distanceCalculation(const fcl::CollisionObject* o1, const fcl::CollisionObject* o2, fcl::DistanceResult& result)
{
    fcl::DistanceRequest request(true);

    //FCL_REAL distance(const CollisionObject* o1, const CollisionObject* o2, const DistanceRequest& request, DistanceResult& result)
    distance(o1, o2, request, result);
}
#endif

#ifdef USE_BULLET
void CollisionAvoidance::visualizeCollisionModel(RobotState::CollisionBody collisionBody,int id) const
{
    const btTransform& transform = collisionBody.bt_transform;
    std::string type = collisionBody.collision_shape.shape_type;
    std::string frame_id = "/map";

    double x = collisionBody.collision_shape.dimensions.x;
    double y = collisionBody.collision_shape.dimensions.y;
    double z = collisionBody.collision_shape.dimensions.z;

    visualization_msgs::Marker modelviz;
    visualization_msgs::MarkerArray marker_array;

    if (type == "Box")
    {
        modelviz.type = visualization_msgs::Marker::CUBE;
        modelviz.header.frame_id = frame_id;
        modelviz.header.stamp = ros::Time::now();
        modelviz.id = id;

        modelviz.scale.x = 2*x;
        modelviz.scale.y = 2*y;
        modelviz.scale.z = 2*z;

        modelviz.pose.position.x = transform.getOrigin().getX();
        modelviz.pose.position.y = transform.getOrigin().getY();
        modelviz.pose.position.z = transform.getOrigin().getZ();

        modelviz.pose.orientation.x = transform.getRotation().getX();
        modelviz.pose.orientation.y = transform.getRotation().getY();
        modelviz.pose.orientation.z = transform.getRotation().getZ();
        modelviz.pose.orientation.w = transform.getRotation().getW();

        modelviz.color.a = 0.5;
        modelviz.color.r = 0;
        modelviz.color.g = 0;
        modelviz.color.b = 1;

        marker_array.markers.push_back(modelviz);

        pub_model_marker_.publish(marker_array);
    }

    else if (type == "Sphere")
    {
        modelviz.type = visualization_msgs::Marker::SPHERE;
        modelviz.header.frame_id = frame_id;
        modelviz.header.stamp = ros::Time::now();
        modelviz.id = id;

        modelviz.scale.x = 2*x;
        modelviz.scale.y = 2*y;
        modelviz.scale.z = 2*z;

        modelviz.pose.position.x = transform.getOrigin().getX();
        modelviz.pose.position.y = transform.getOrigin().getY();
        modelviz.pose.position.z = transform.getOrigin().getZ();
        modelviz.pose.orientation.x = transform.getRotation().getX();
        modelviz.pose.orientation.y = transform.getRotation().getY();
        modelviz.pose.orientation.z = transform.getRotation().getZ();
        modelviz.pose.orientation.w = transform.getRotation().getW();

        modelviz.color.a = 0.5;
        modelviz.color.r = 0;
        modelviz.color.g = 0;
        modelviz.color.b = 1;

        marker_array.markers.push_back(modelviz);

        pub_model_marker_.publish(marker_array);
    }

    else if (type == "CylinderY")
    {
        modelviz.type = visualization_msgs::Marker::CYLINDER;
        modelviz.header.frame_id = frame_id;
        modelviz.header.stamp = ros::Time::now();
        modelviz.id = id;

        modelviz.scale.x = 2*x;
        modelviz.scale.y = 2*z;
        modelviz.scale.z = 2*y;

        modelviz.pose.position.x = transform.getOrigin().getX();
        modelviz.pose.position.y = transform.getOrigin().getY();
        modelviz.pose.position.z = transform.getOrigin().getZ();

        btTransform rotXfromZtoY;
        btTransform vizfromZtoY;
        rotXfromZtoY.setOrigin(btVector3(0.0, 0.0, 0.0));
        rotXfromZtoY.setRotation(btQuaternion(-sqrt(0.5),0.0,0.0,sqrt(0.5)));
        vizfromZtoY.mult(transform,rotXfromZtoY);

        modelviz.pose.orientation.x = vizfromZtoY.getRotation().getX();
        modelviz.pose.orientation.y = vizfromZtoY.getRotation().getY();
        modelviz.pose.orientation.z = vizfromZtoY.getRotation().getZ();
        modelviz.pose.orientation.w = vizfromZtoY.getRotation().getW();

        modelviz.color.a = 0.5;
        modelviz.color.r = 0;
        modelviz.color.g = 0;
        modelviz.color.b = 1;

        marker_array.markers.push_back(modelviz);

        pub_model_marker_.publish(marker_array);
    }

    else if (type == "CylinderZ")
    {
        modelviz.type = visualization_msgs::Marker::CYLINDER;
        modelviz.header.frame_id = frame_id;
        modelviz.header.stamp = ros::Time::now();
        modelviz.id = id;

        modelviz.scale.x = 2*x;
        modelviz.scale.y = 2*y;
        modelviz.scale.z = 2*z;

        modelviz.pose.position.x = transform.getOrigin().getX();
        modelviz.pose.position.y = transform.getOrigin().getY();
        modelviz.pose.position.z = transform.getOrigin().getZ();
        modelviz.pose.orientation.x = transform.getRotation().getX();
        modelviz.pose.orientation.y = transform.getRotation().getY();
        modelviz.pose.orientation.z = transform.getRotation().getZ();
        modelviz.pose.orientation.w = transform.getRotation().getW();

        modelviz.color.a = 0.5;
        modelviz.color.r = 0;
        modelviz.color.g = 0;
        modelviz.color.b = 1;

        marker_array.markers.push_back(modelviz);

        pub_model_marker_.publish(marker_array);
    }

    else if (type == "Cone")
    {
        modelviz.type = visualization_msgs::Marker::MESH_RESOURCE;
        modelviz.header.frame_id = frame_id;
        modelviz.header.stamp = ros::Time::now();
        modelviz.mesh_resource = "package://amigo_whole_body_controller/data/cone.dae";
        modelviz.id = id;

        // ToDo:: Get rid of hardcoded bug fix
        modelviz.scale.x = 2.0*x; // there was 2.3 here, from the scale of cone.dae?
        modelviz.scale.y = 2.0*z; // 3.0
        modelviz.scale.z = 2.0*y; // 2.3

        btTransform rotXfromZtoY;
        btTransform fromZto;
        rotXfromZtoY.setOrigin(btVector3(0.0, 0.0, 0.0));
        rotXfromZtoY.setRotation(btQuaternion(sqrt(0.5),0.0,0.0,sqrt(0.5)));
        fromZto.mult(transform,rotXfromZtoY);

        modelviz.pose.position.x = transform.getOrigin().getX();
        modelviz.pose.position.y = transform.getOrigin().getY();
        modelviz.pose.position.z = transform.getOrigin().getZ();
        modelviz.pose.orientation.x = fromZto.getRotation().getX();
        modelviz.pose.orientation.y = fromZto.getRotation().getY();
        modelviz.pose.orientation.z = fromZto.getRotation().getZ();
        modelviz.pose.orientation.w = fromZto.getRotation().getW();

        modelviz.color.a = 0.5;
        modelviz.color.r = 0;
        modelviz.color.g = 0;
        modelviz.color.b = 1;

        marker_array.markers.push_back(modelviz);

        pub_model_marker_.publish(marker_array);
    }
}
#endif
#ifdef USE_FCL
void CollisionAvoidance::visualizeCollisionModelFCL(RobotState::CollisionBody collisionBody,int id) const
{
    //const fcl::CollisionGeometry *cg = collisionBody.fcl_shape.get();
    const fcl::CollisionObject &obj = *collisionBody.fcl_object.get();

    visualization_msgs::Marker m;
    vwm::objectFCLtoMarker(obj, m);
    m.id = id;
    m.header.frame_id = "/map";

    m.color.a = 0.5;
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 1;

    pub_model_marker_fcl_.publish(m);
}
#endif

#ifdef USE_BULLET
void CollisionAvoidance::pickMinimumDistance(std::vector<Distance> &calculatedDistances, std::vector<Distance> &minimumDistances)
{
    // For a single collision body pick the minimum of all calculated closest distances by the GJK algorithm.
    Distance dmin;
    dmin.bt_distance.m_distance = 10000.0;
    for (std::vector<Distance>::iterator itrDistance = calculatedDistances.begin(); itrDistance != calculatedDistances.end(); ++itrDistance)
    {
        Distance &distance = *itrDistance;
        if ( distance.bt_distance.m_distance < dmin.bt_distance.m_distance )
        {
            dmin = distance;
        }
    }

    // Store all minimum distances;
    if ( dmin.bt_distance.m_distance < 100)
    {
        minimumDistances.push_back(dmin);
    }

}
#endif

#ifdef USE_FCL
void CollisionAvoidance::pickMinimumDistance(std::vector<Distance2> &calculatedDistances, std::vector<Distance2> &minimumDistances)
{
    // For a single collision body pick the minimum of all calculated closest distances by the GJK algorithm.
    Distance2 dmin;
    dmin.result.min_distance = 10000.0;
    for (std::vector<Distance2>::iterator itrDistance = calculatedDistances.begin(); itrDistance != calculatedDistances.end(); ++itrDistance)
    {
        Distance2 &distance = *itrDistance;
        if (distance.result.min_distance < dmin.result.min_distance)
        {
            dmin = distance;
        }
    }

    // Store all minimum distances;
    if ( dmin.result.min_distance < 100)
    {
        minimumDistances.push_back(dmin);
    }
}
#endif

#ifdef USE_BULLET
void CollisionAvoidance::calculateRepulsiveForce(const std::vector<Distance> &minimumDistances, std::vector<RepulsiveForce> &repulsiveForces, const collisionAvoidanceParameters::Parameters &param)
{
    RepulsiveForce F;
    for (std::vector<Distance>::const_iterator itrMinDist = minimumDistances.begin(); itrMinDist != minimumDistances.end(); ++itrMinDist)
    {
        const Distance &dmin = *itrMinDist;
        if (dmin.bt_distance.m_distance <= param.d_threshold )
        {
            F.frame_id = dmin.frame_id;

            F.direction = Eigen::Vector3d(
                        dmin.bt_distance.m_normalOnBInWorld.getX(),
                        dmin.bt_distance.m_normalOnBInWorld.getY(),
                        dmin.bt_distance.m_normalOnBInWorld.getZ());

            btVector3 pointOnA = dmin.bt_distance.m_pointInWorld + dmin.bt_distance.m_distance * dmin.bt_distance.m_normalOnBInWorld;
            F.pointOnA = Eigen::Vector3d(
                        pointOnA.getX(),
                        pointOnA.getY(),
                        pointOnA.getZ());

            F.amplitude = param.f_max / pow(param.d_threshold,param.order) * pow((param.d_threshold - dmin.bt_distance.m_distance),param.order) ;

            // Store all minimum distances;
            repulsiveForces.push_back(F);
        }
    }
}
#endif
#ifdef USE_FCL
void CollisionAvoidance::calculateRepulsiveForce(const std::vector<Distance2> &minimumDistances, std::vector<RepulsiveForce> &repulsiveForces, const collisionAvoidanceParameters::Parameters &param)
{
    RepulsiveForce F;

    for (std::vector<Distance2>::const_iterator itrMinDist = minimumDistances.begin(); itrMinDist != minimumDistances.end(); ++itrMinDist)
    {
        const Distance2 &dmin = *itrMinDist;

        if (dmin.result.min_distance <= param.d_threshold)
        {
            Eigen::Vector3d p0(dmin.result.nearest_points[0][0],
                               dmin.result.nearest_points[0][1],
                               dmin.result.nearest_points[0][2]);
            Eigen::Vector3d p1(dmin.result.nearest_points[1][0],
                               dmin.result.nearest_points[1][1],
                               dmin.result.nearest_points[1][2]);

            F.frame_id  = dmin.frame_id;

            // The vector must point into the opposite direction of
            // the vector from the current object to the other object
            // - (pOther - pCurrent) = - (p1 - p0)
            F.direction = p0 - p1;
            F.direction.normalize();
            F.pointOnA  = p0;
            F.amplitude = param.f_max / pow(param.d_threshold,param.order) * pow((param.d_threshold - dmin.result.min_distance),param.order);

            // Store all minimum distances;
            repulsiveForces.push_back(F);
        }
    }
}
#endif

void CollisionAvoidance::visualizeRepulsiveForce(Distance2 &d_min,int id) const
{
#ifdef USE_FCL
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker RFviz;
    geometry_msgs::Point pA;
    geometry_msgs::Point pB;

    RFviz.type = visualization_msgs::Marker::ARROW;
    RFviz.header.frame_id = "/map"; //"/base_link";
    RFviz.header.stamp = ros::Time::now();
    RFviz.id = id;

    RFviz.scale.x = 0.02;
    RFviz.scale.y = 0.04;
    RFviz.scale.z = 0.04;

    pA.x = d_min.result.nearest_points[0][0];
    pA.y = d_min.result.nearest_points[0][1];
    pA.z = d_min.result.nearest_points[0][2];

    pB.x = d_min.result.nearest_points[1][0];
    pB.y = d_min.result.nearest_points[1][1];
    pB.z = d_min.result.nearest_points[1][2];

    RFviz.points.push_back(pA);
    RFviz.points.push_back(pB);


    if (d_min.result.min_distance <= ca_param_.self_collision.d_threshold )
    {
        RFviz.color.a = 1;
        RFviz.color.r = 1;
        RFviz.color.g = 0;
        RFviz.color.b = 0;
    }
    else if (d_min.result.min_distance > ca_param_.self_collision.d_threshold )
    {
        RFviz.color.a = 1;
        RFviz.color.r = 0;
        RFviz.color.g = 1;
        RFviz.color.b = 0;
    }

    RFviz.lifetime = ros::Duration(1.0);

    marker_array.markers.push_back(RFviz);

    pub_forces_marker_fcl_.publish(marker_array);
#endif
}

void CollisionAvoidance::visualizeRepulsiveForce(Distance &d_min,int id) const
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker RFviz;
    geometry_msgs::Point pA;
    geometry_msgs::Point pB;

    RFviz.type = visualization_msgs::Marker::ARROW;
    RFviz.header.frame_id = "/map"; //"/base_link";
    RFviz.header.stamp = ros::Time::now();
    RFviz.id = id;

    RFviz.scale.x = 0.02;
    RFviz.scale.y = 0.04;
    RFviz.scale.z = 0.04;

    pA.x = d_min.bt_distance.m_pointInWorld.getX();
    pA.y = d_min.bt_distance.m_pointInWorld.getY();
    pA.z = d_min.bt_distance.m_pointInWorld.getZ();

    pB.x = d_min.bt_distance.m_pointInWorld.getX() + d_min.bt_distance.m_distance * d_min.bt_distance.m_normalOnBInWorld.getX();
    pB.y = d_min.bt_distance.m_pointInWorld.getY() + d_min.bt_distance.m_distance * d_min.bt_distance.m_normalOnBInWorld.getY();
    pB.z = d_min.bt_distance.m_pointInWorld.getZ() + d_min.bt_distance.m_distance * d_min.bt_distance.m_normalOnBInWorld.getZ();

    RFviz.points.push_back(pA);
    RFviz.points.push_back(pB);


    if (d_min.bt_distance.m_distance <= ca_param_.self_collision.d_threshold )
    {
        RFviz.color.a = 1;
        RFviz.color.r = 1;
        RFviz.color.g = 0;
        RFviz.color.b = 0;
    }
    else if (d_min.bt_distance.m_distance > ca_param_.self_collision.d_threshold )
    {
        RFviz.color.a = 1;
        RFviz.color.r = 0;
        RFviz.color.g = 1;
        RFviz.color.b = 0;
    }

    RFviz.lifetime = ros::Duration(1.0);

    marker_array.markers.push_back(RFviz);

    pub_forces_marker_.publish(marker_array);
}


void CollisionAvoidance::visualizeBBX(octomath::Vector3 min, octomath::Vector3 max, int id) const
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker BBXviz;

    BBXviz.type = visualization_msgs::Marker::CUBE;
    BBXviz.header.frame_id = "/map"; //"/base_link";
    BBXviz.header.stamp = ros::Time::now();
    BBXviz.id = id;

    BBXviz.scale.x = max(0) - min(0);
    BBXviz.scale.y = max(1) - min(1);
    BBXviz.scale.z = max(2) - min(2);

    BBXviz.pose.position.x = min(0) + (max(0) - min(0))/2;
    BBXviz.pose.position.y = min(1) + (max(1) - min(1))/2;
    BBXviz.pose.position.z = min(2) + (max(2) - min(2))/2;

    BBXviz.pose.orientation.x = 0;
    BBXviz.pose.orientation.y = 0;
    BBXviz.pose.orientation.z = 0;
    BBXviz.pose.orientation.w = 1;

    BBXviz.color.a = 0.25;
    BBXviz.color.r = 1;
    BBXviz.color.g = 1;
    BBXviz.color.b = 1;

    marker_array.markers.push_back(BBXviz);

    pub_bbx_marker_.publish(marker_array);

}


void CollisionAvoidance::setOctoMap(octomap::OcTreeStamped* octree)
{  
    delete octomap_;
    octomap_ = octree;
}


void CollisionAvoidance::findOuterPoints(RobotState::CollisionBody& collisionBody, btVector3 &min, btVector3 &max)
{
    // Calculate all outer points of the axis aligned BBX
    std::vector<btVector3> outer_points;
    btVector3 point1 = collisionBody.bt_transform.getOrigin() + collisionBody.bt_transform.getBasis() * btVector3 (collisionBody.collision_shape.dimensions.x,
                                                                                                                   collisionBody.collision_shape.dimensions.y,
                                                                                                                   collisionBody.collision_shape.dimensions.z);
    outer_points.push_back(point1);

    btVector3 point2 = collisionBody.bt_transform.getOrigin() + collisionBody.bt_transform.getBasis() * btVector3 (-collisionBody.collision_shape.dimensions.x,
                                                                                                                   collisionBody.collision_shape.dimensions.y,
                                                                                                                   collisionBody.collision_shape.dimensions.z);
    outer_points.push_back(point2);

    btVector3 point3 = collisionBody.bt_transform.getOrigin() + collisionBody.bt_transform.getBasis() * btVector3 (collisionBody.collision_shape.dimensions.x,
                                                                                                                   -collisionBody.collision_shape.dimensions.y,
                                                                                                                   collisionBody.collision_shape.dimensions.z);
    outer_points.push_back(point3);

    btVector3 point4 = collisionBody.bt_transform.getOrigin() + collisionBody.bt_transform.getBasis() * btVector3 (collisionBody.collision_shape.dimensions.x,
                                                                                                                   collisionBody.collision_shape.dimensions.y,
                                                                                                                   -collisionBody.collision_shape.dimensions.z);
    outer_points.push_back(point4);

    btVector3 point5 = collisionBody.bt_transform.getOrigin() + collisionBody.bt_transform.getBasis() * btVector3 (-collisionBody.collision_shape.dimensions.x,
                                                                                                                   -collisionBody.collision_shape.dimensions.y,
                                                                                                                   collisionBody.collision_shape.dimensions.z);
    outer_points.push_back(point5);

    btVector3 point6 = collisionBody.bt_transform.getOrigin() + collisionBody.bt_transform.getBasis() * btVector3 (collisionBody.collision_shape.dimensions.x,
                                                                                                                   -collisionBody.collision_shape.dimensions.y,
                                                                                                                   -collisionBody.collision_shape.dimensions.z);
    outer_points.push_back(point6);

    btVector3 point7 = collisionBody.bt_transform.getOrigin() + collisionBody.bt_transform.getBasis() * btVector3 (-collisionBody.collision_shape.dimensions.x,
                                                                                                                   collisionBody.collision_shape.dimensions.y,
                                                                                                                   -collisionBody.collision_shape.dimensions.z);
    outer_points.push_back(point7);

    btVector3 point8 = collisionBody.bt_transform.getOrigin() + collisionBody.bt_transform.getBasis() * btVector3 (-collisionBody.collision_shape.dimensions.x,
                                                                                                                   -collisionBody.collision_shape.dimensions.y,
                                                                                                                   -collisionBody.collision_shape.dimensions.z);
    outer_points.push_back(point8);

    // Determin the min and max point for the axis aligned BBX
    for (unsigned int i = 0; i < outer_points.size(); i++)
    {
        for (unsigned int j = 0; j < 3; j++)
        {
            if (min[j] > outer_points[i][j]) {
                min[j] = outer_points[i][j];
            }
            if (max[j] < outer_points[i][j]) {
                max[j] = outer_points[i][j];
            }
        }
    }
}

void CollisionAvoidance::removeOctomapBBX(const geometry_msgs::Point& goal, const std::string& root){

    if (octomap_){
        if (octomap_->size() > 0){
            std::map<std::string, KDL::Frame>::iterator itrRF = robot_state_->fk_poses_.find(root);
            KDL::Frame Frame_map_root = itrRF->second;
            KDL::Frame Frame_root_goal;
            Frame_root_goal.p.x(goal.x);
            Frame_root_goal.p.y(goal.y);
            Frame_root_goal.p.z(goal.z);

            // Convert goal pose to map
            KDL::Frame Frame_map_goal = Frame_map_root*Frame_root_goal;
            ROS_INFO("Collision Avoidance: Bounding box removed from octomap in /map (%f,%f,%f)",Frame_map_goal.p.x(),Frame_map_goal.p.y(),Frame_map_goal.p.z());

            // Set up bounding box dimension
            geometry_msgs::Point bbx_min;
            geometry_msgs::Point bbx_max;

            bbx_max.x = Frame_map_goal.p.x() + 0.1;
            bbx_max.y = Frame_map_goal.p.y() + 0.1;
            bbx_max.z = Frame_map_goal.p.z() + 0.1;
            bbx_min.x = Frame_map_goal.p.x() - 0.1;
            bbx_min.y = Frame_map_goal.p.y() - 0.1;
            bbx_min.z = Frame_map_goal.p.z() - 0.1;

            ROS_INFO("Bounding box max: (%f %f %f), min (%f %f %f)",bbx_max.x,bbx_max.y,bbx_max.z,bbx_min.x, bbx_min.y, bbx_min.z );

            octomap::point3d min(bbx_min.x, bbx_min.y, bbx_min.z);
            octomap::point3d max(bbx_max.x, bbx_max.y, bbx_max.z);

            for(OctreeType::leaf_bbx_iterator it = octomap_->begin_leafs_bbx(min,max),
                end=octomap_->end_leafs_bbx(); it!= end; ++it){

                it->setLogOdds(octomap::logodds(0.0));
            }
            octomap_->updateInnerOccupancy();
        }
    }
}

} // namespace
