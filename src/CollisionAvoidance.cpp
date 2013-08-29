#include "CollisionAvoidance.h"
#include <math.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "profiling/Profiler.h"

using namespace std;

CollisionAvoidance::CollisionAvoidance(collisionAvoidanceParameters &parameters, const double Ts)
    : ca_param_(parameters), Ts_ (Ts), octomap_(NULL)
{
}

CollisionAvoidance::~CollisionAvoidance()
{
}

bool CollisionAvoidance::initialize(RobotState &robotstate)
{
    robot_state_ = &robotstate;

    ROS_INFO_STREAM("Initializing Obstacle Avoidance, ");

    // Get node handle
    ros::NodeHandle n("~");
    std::string ns = ros::this_node::getName();

    // Initialize output topics
    pub_model_marker_ = n.advertise<visualization_msgs::MarkerArray>("/whole_body_controller/collision_model_markers/", 10);
    pub_forces_marker_ = n.advertise<visualization_msgs::MarkerArray>("/whole_body_controller/repulsive_forces_markers/", 10);
    pub_bbx_marker_ = n.advertise<visualization_msgs::MarkerArray>("/whole_body_controller/bbx_markers/", 10);
    pub_rep_force_ = n.advertise<std_msgs::Float64>("/whole_body_controller/rep_force/", 10);
    pub_d_min_ = n.advertise<std_msgs::Float64>("/whole_body_controller/d_min/", 10);
    pub_CA_wrench_ = n.advertise<std_msgs::Float64MultiArray>("/whole_body_controller/ca_wrench/", 10);
    pub_delta_p_ = n.advertise<std_msgs::Float64MultiArray>("/whole_body_controller/delta_p/", 10);

    // Initialize OctoMap
    octomap_ = new octomap::OcTree(ca_param_.environment_collision.octomap_resolution);

    // Initialize a frame that indicates no fix is required from a FK pose to the collision shape
    no_fix_.p.x(0);
    no_fix_.p.y(0);
    no_fix_.p.z(0);
    no_fix_.M = KDL::Rotation::Quaternion(0,0,0,1);

    initializeCollisionModel(robotstate);

    ROS_INFO_STREAM("Initialized Obstacle Avoidance");

    return true;
}

void CollisionAvoidance::apply(RobotState &robotstate)
{
    //ThreadProfiler::Start("CA");
    robot_state_ = &robotstate;
    calculateTransform();

    // Calculate the wrenches as a result of (self-)collision avoidance
    std::vector<Distance> min_distances_total;
    std::vector<RepulsiveForce> repulsive_forces_total;
    std::vector<Wrench> wrenches_total;
    repulsive_forces_total.clear();
    wrenches_total.clear();

    // Calculate the repulsive forces as a result of the self-collision avoidance.
    selfCollision(min_distances_total,repulsive_forces_total);

    // Create topics for plots
    for (std::vector<Distance>::iterator itr_d_min = min_distances_total.begin(); itr_d_min != min_distances_total.end(); ++itr_d_min)
    {
        Distance dist = *itr_d_min;
        if (dist.frame_id == "grippoint_right")
        {
            double distance = dist.bt_distance.m_distance;
            pub_d_min_.publish(distance);
            //std::cout << " d_min = " << distance << std::endl;
        }
    }
    for (std::vector<RepulsiveForce>::iterator itr_f_rep = repulsive_forces_total.begin(); itr_f_rep != repulsive_forces_total.end(); ++itr_f_rep)
    {
        RepulsiveForce f_rep = *itr_f_rep;
        if (f_rep.frame_id == "grippoint_right")
        {
            double force = f_rep.amplitude;
            pub_rep_force_.publish(force);
            //std::cout << " F_rep = " << force << std::endl;
        }
    }

    // Calculate the repulsive forces as a result of the environment collision avoidance.
    if (octomap_->size() > 0)
    {
        environmentCollision(min_distances_total,repulsive_forces_total);
    }

    calculateWrenches(repulsive_forces_total, wrenches_total);

    /// Output
    visualize(min_distances_total);
    outputWrenches(wrenches_total);
    //ThreadProfiler::Stop("CA");
}


void CollisionAvoidance::selfCollision(std::vector<Distance> &min_distances, std::vector<RepulsiveForce> &repulsive_forces)
{
    active_groups_.clear();
    collision_groups_.clear();
    for (std::vector< std::vector<RobotState::CollisionBody> >::iterator itrGroups = robot_state_->robot_.groups.begin(); itrGroups != robot_state_->robot_.groups.end(); ++itrGroups)
    {
        std::vector<RobotState::CollisionBody> &group = *itrGroups;

        // Determine if group has tip frame
        // ToDo: get rid of hardcoding
        bool group_check = false;
        for (std::vector<RobotState::CollisionBody>::iterator itrBodies = group.begin(); itrBodies != group.end(); ++itrBodies)
        {
            RobotState::CollisionBody &collisionBody = *itrBodies;
            //std::cout << collisionBody.bt_shape->getName() << std::endl;
            if ( collisionBody.frame_id == "grippoint_left"
                 || collisionBody.frame_id == "grippoint_right")
            {
                group_check = true;
            }
        }

        if ( group_check == true )
        {
            active_groups_.push_back(group);
        }
        else if ( group_check == false )
        {
            // IF NOT FOUND collision_group
            collision_groups_.push_back(group);
        }
    }

    for (std::vector< std::vector<RobotState::CollisionBody> >::iterator itrActiveGroup = active_groups_.begin(); itrActiveGroup != active_groups_.end(); ++itrActiveGroup)
    {
        std::vector<Distance> distanceCollection;
        std::vector<RobotState::CollisionBody> &activeGroup = *itrActiveGroup;
        for (std::vector<RobotState::CollisionBody>::iterator itrActiveBody = activeGroup.begin(); itrActiveBody != activeGroup.end(); ++itrActiveBody)
        {
            RobotState::CollisionBody &currentBody = *itrActiveBody;

            // Distance check with other active groups
            for (std::vector< std::vector<RobotState::CollisionBody> >::iterator itrActiveGroup = active_groups_.begin(); itrActiveGroup != active_groups_.end(); ++itrActiveGroup)
            {
                std::vector<RobotState::CollisionBody> &checkGroup = *itrActiveGroup;
                if (checkGroup.front().name_collision_body != activeGroup.front().name_collision_body)
                {
                    for (std::vector<RobotState::CollisionBody>::iterator itrCheckBody = checkGroup.begin(); itrCheckBody != checkGroup.end(); ++itrCheckBody)
                    {
                        RobotState::CollisionBody &checkBody = *itrCheckBody;

                        // Check for exclusions
                        bool skip_check = false;
                        for (std::vector<RobotState::Exclusion> ::iterator itrExcl = robot_state_->exclusion_checks.checks.begin(); itrExcl != robot_state_->exclusion_checks.checks.end(); ++itrExcl)
                        {
                            RobotState::Exclusion &excluded_bodies = *itrExcl;
                            if ( currentBody.name_collision_body == excluded_bodies.name_body_A
                                 && checkBody.name_collision_body == excluded_bodies.name_body_B )
                            {
                                skip_check = true;
                            }
                        }

                        if (skip_check == false)
                        {
                            Distance distance;
                            distance.frame_id = currentBody.frame_id;
                            distanceCalculation(*currentBody.bt_shape,*checkBody.bt_shape,currentBody.bt_transform,checkBody.bt_transform,distance.bt_distance);
                            distanceCollection.push_back(distance);
                        }
                    }
                }
            }

            // Distance check with collision groups
            for (std::vector<std::vector<RobotState::CollisionBody> >::iterator itrCollisionGroup = collision_groups_.begin(); itrCollisionGroup != collision_groups_.end(); ++itrCollisionGroup)
            {
                std::vector<RobotState::CollisionBody> &collisionGroup = *itrCollisionGroup;
                //std::cout << "size collisionGroup = " << collisionGroup.size() << std::endl;
                for (std::vector<RobotState::CollisionBody>::iterator itrCollisionBodies = collisionGroup.begin(); itrCollisionBodies != collisionGroup.end(); ++itrCollisionBodies)
                {
                    RobotState::CollisionBody &collisionBody = *itrCollisionBodies;

                    // Check for exclusions
                    bool skip_check = false;
                    for (std::vector<RobotState::Exclusion> ::iterator itrExcl = robot_state_->exclusion_checks.checks.begin(); itrExcl != robot_state_->exclusion_checks.checks.end(); ++itrExcl)
                    {
                        RobotState::Exclusion &excluded_bodies = *itrExcl;

                        if ( currentBody.name_collision_body == excluded_bodies.name_body_A
                             && collisionBody.name_collision_body == excluded_bodies.name_body_B )
                        {
                            skip_check = true;
                        }
                    }

                    if (skip_check == false)
                    {
                        Distance distance;
                        distance.frame_id = currentBody.frame_id;
                        distanceCalculation(*currentBody.bt_shape,*collisionBody.bt_shape,currentBody.bt_transform,collisionBody.bt_transform,distance.bt_distance);
                        distanceCollection.push_back(distance);

                    }
                }
            }
            // Find minimum distance
            pickMinimumDistance(distanceCollection,min_distances);
        }
    }
    // Calculate the repulsive forces
    calculateRepulsiveForce(min_distances,repulsive_forces,ca_param_.self_collision);
}


void CollisionAvoidance::environmentCollision(std::vector<Distance> &min_distances, std::vector<RepulsiveForce> &repulsive_forces)
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

            // Add the threshold distance to the found axis alligned BBX
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
            for(octomap::OcTree::leaf_bbx_iterator it = octomap_->begin_leafs_bbx(min,max), end=octomap_->end_leafs_bbx(); it!= end; ++it)
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
            }
            // Find minimum distance
            pickMinimumDistance(distanceCollection,min_distances);
        }
    }
    // Calculate the repulsive forces
    calculateRepulsiveForce(min_distances,repulsive_forces,ca_param_.environment_collision);
}


void CollisionAvoidance::calculateWrenches(std::vector<RepulsiveForce> &repulsive_forces, std::vector<Wrench> &wrenches_out)
{
    for (std::vector<RepulsiveForce>::iterator itrRF = repulsive_forces.begin(); itrRF != repulsive_forces.end(); ++itrRF)
    {
        RepulsiveForce &RF = *itrRF;
        Wrench W;
        KDL::Frame p0;
        Eigen::VectorXd wrench_calc(6);
        wrench_calc.setZero();
        double dpx, dpy, dpz, fx, fy, fz;

        // Find the frame of the kinematic model on which the wrench acts.
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

        // Calculate delta p
        dpx = RF.pointOnA.getX() - p0.p.x();
        dpy = RF.pointOnA.getY() - p0.p.y();
        dpz = RF.pointOnA.getZ() - p0.p.z();

        // Calculate the force components
        fx = RF.amplitude*RF.direction.getX();
        fy = RF.amplitude*RF.direction.getY();
        fz = RF.amplitude*RF.direction.getZ();

        wrench_calc[0] = fx;
        wrench_calc[1] = fy;
        wrench_calc[2] = fz;
        wrench_calc[3] = fz*dpy - fy*dpz;
        wrench_calc[4] = fx*dpz - fz*dpx;
        wrench_calc[5] = fy*dpx - fx*dpy;

        if (RF.frame_id == "grippoint_left")
        {
            // Publish the wrench
            std_msgs::Float64MultiArray msgP;
            msgP.data.push_back(dpx);
            msgP.data.push_back(dpy);
            msgP.data.push_back(dpz);

            pub_delta_p_.publish(msgP);
        }

        W.frame_id = RF.frame_id;
        W.wrench = wrench_calc;

        wrenches_out.push_back(W);
    }
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
            visualizeCollisionModel(collisionBody,id++);
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

            if (type=="Box")
            {
                collisionBody.bt_shape = new btBoxShape(btVector3(x,y,z));
            }
            else if (type == "Sphere")
            {
                collisionBody.bt_shape = new btSphereShape(x);
            }
            else if (type == "Cone")
            {
                collisionBody.bt_shape = new btConeShapeZ(x-0.05,2*z);
            }
            else if (type == "CylinderY")
            {
                collisionBody.bt_shape = new btCylinderShape(btVector3(x,y,z));
            }
            else if (type == "CylinderZ")
            {
                collisionBody.bt_shape = new btCylinderShapeZ(btVector3(x,y,z));
            }
        }
    }
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

            setTransform(collisionBody.fk_pose, collisionBody.fix_pose, collisionBody.bt_transform);
        }
    }
}


void CollisionAvoidance::setTransform(KDL::Frame &fkPose, KDL::Frame &fixPose, btTransform &transform_out)
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


void CollisionAvoidance::distanceCalculation(btConvexShape& shapeA, btConvexShape& shapeB, btTransform& transformA, btTransform& transformB, btPointCollector &distance_out)
{
    // Set solvers
    btConvexPenetrationDepthSolver*	depthSolver = new btMinkowskiPenetrationDepthSolver;
    btSimplexSolverInterface* simplexSolver = new btVoronoiSimplexSolver;
    btGjkPairDetector convexConvex(&shapeA, &shapeB, simplexSolver, depthSolver);

    // Set input transforms
    btGjkPairDetector::ClosestPointInput input;
    input.m_transformA = transformA;
    input.m_transformB = transformB;

    // Calculate closest distance
    convexConvex.getClosestPoints(input, distance_out, 0);
}


void CollisionAvoidance::visualizeCollisionModel(RobotState::CollisionBody collisionBody,int id) const
{
    const btTransform& transform = collisionBody.bt_transform;
    std::string type = collisionBody.collision_shape.shape_type;
    string frame_id = "map";

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
        modelviz.scale.x = 2.3*x;
        modelviz.scale.y = 3.0*z;
        modelviz.scale.z = 2.3*y;

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
    minimumDistances.push_back(dmin);
}


void CollisionAvoidance::calculateRepulsiveForce(std::vector<Distance> &minimumDistances, std::vector<RepulsiveForce> &repulsiveForces, collisionAvoidanceParameters::Parameters &param)
{
    RepulsiveForce F;
    for (std::vector<Distance>::iterator itrMinDist = minimumDistances.begin(); itrMinDist != minimumDistances.end(); ++itrMinDist)
    {
        Distance &dmin = *itrMinDist;
        if (dmin.bt_distance.m_distance <= param.d_threshold )
        {
            F.frame_id = dmin.frame_id;
            F.direction = dmin.bt_distance.m_normalOnBInWorld;
            F.pointOnA = dmin.bt_distance.m_pointInWorld + dmin.bt_distance.m_distance * dmin.bt_distance.m_normalOnBInWorld;
            F.amplitude = param.f_max / pow(param.d_threshold,param.order) * pow((param.d_threshold - dmin.bt_distance.m_distance),param.order) ;

            // Store all minimum distances;
            repulsiveForces.push_back(F);
        }
    }
}


void CollisionAvoidance::outputWrenches(std::vector<Wrench> &wrenches)
{
    for (std::vector<Wrench>::iterator itrW = wrenches.begin(); itrW != wrenches.end(); ++itrW)
    {
        Wrench W = *itrW;
        robot_state_->tree_.addCartesianWrench(W.frame_id,W.wrench);

        if (W.frame_id == "grippoint_right")
        {
            // Publish the wrench
            std_msgs::Float64MultiArray msgW;
            for (uint i = 0; i < W.wrench.rows(); i++)
            {
                msgW.data.push_back(W.wrench(i));
            }
            pub_CA_wrench_.publish(msgW);
        }
    }
}


void CollisionAvoidance::visualizeRepulsiveForce(Distance &d_min,int id) const
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker RFviz;
    geometry_msgs::Point pA;
    geometry_msgs::Point pB;

    RFviz.type = visualization_msgs::Marker::ARROW;
    RFviz.header.frame_id = "map"; //"/base_link";
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

    //if(pA.x > 0.001 && pA.y > 0.001 && pB.x > 0.001 && pB.y > 0.001)
    //{
    RFviz.points.push_back(pA);
    RFviz.points.push_back(pB);
    //}


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

    marker_array.markers.push_back(RFviz);

    pub_forces_marker_.publish(marker_array);
}


void CollisionAvoidance::visualizeBBX(octomath::Vector3 min, octomath::Vector3 max, int id) const
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker BBXviz;

    BBXviz.type = visualization_msgs::Marker::CUBE;
    BBXviz.header.frame_id = "map"; //"/base_link";
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


void CollisionAvoidance::setOctoMap(octomap::OcTree* octree)
{
    octomap_ = octree;
}


void CollisionAvoidance::findOuterPoints(RobotState::CollisionBody& collisionBody, btVector3 &min, btVector3 &max)
{
    // Calculate all outer points of the axis alligned BBX
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
