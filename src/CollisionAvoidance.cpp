#include "CollisionAvoidance.h"
#include <math.h>

#include "visualization_msgs/MarkerArray.h"

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

    pub_marker_ = n.advertise<visualization_msgs::MarkerArray>("/whole_body_controller/collision_avoidance_markers/", 10);

    //octomap_ = new octomap::OcTree(ca_param_.environment_collision.octomap_resolution);

    no_fix_.header.frame_id = "none";
    no_fix_.pose.position.x = 0;
    no_fix_.pose.position.y = 0;
    no_fix_.pose.position.z = 0;
    no_fix_.pose.orientation.x = 0;
    no_fix_.pose.orientation.y = 0;
    no_fix_.pose.orientation.z = 0;
    no_fix_.pose.orientation.w = 1;

    initializeCollisionModel(robotstate);

    ROS_INFO_STREAM("Initialized Obstacle Avoidance");

    return true;
}

void CollisionAvoidance::apply(RobotState &robotstate)
{
    ThreadProfiler::Start("CA");
    robot_state_ = &robotstate;
    calculateTransform();

    chain_left_ = robot_state_->chain_left_;
    chain_right_ = robot_state_->chain_right_;

    // Calculate the wrenches as a result of (self-)collision avoidance
    std::vector<Distance> min_distances_total;
    std::vector<ReactionForce> reaction_forces_total;
    std::vector<Wrench> wrenches_total;
    reaction_forces_total.clear();
    wrenches_total.clear();

    selfCollision(min_distances_total,reaction_forces_total);

    if (octomap_->size() > 0)
    {
        environmentCollision(min_distances_total,reaction_forces_total);
    }

    calculateWrenches(reaction_forces_total, wrenches_total);

    // Output
    visualize(min_distances_total);
    outputWrenches(wrenches_total);
    ThreadProfiler::Stop("CA");
}

void CollisionAvoidance::selfCollision(std::vector<Distance> &min_distances, std::vector<ReactionForce> &reaction_forces)
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
            if ( collisionBody.fix_pose.header.frame_id == "grippoint_left"
                 || collisionBody.fix_pose.header.frame_id == "grippoint_right")
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
                            distance.frame_id = currentBody.fix_pose.header.frame_id;
                            //ThreadProfiler::Start("DistanceCalc");
                            distanceCalculation(*currentBody.bt_shape,*checkBody.bt_shape,currentBody.bt_transform,checkBody.bt_transform,distance.bt_distance);
                            distanceCollection.push_back(distance);
                            //ThreadProfiler::Stop("DistanceCalc");
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
                        distance.frame_id = currentBody.fix_pose.header.frame_id;
                        distanceCalculation(*currentBody.bt_shape,*collisionBody.bt_shape,currentBody.bt_transform,collisionBody.bt_transform,distance.bt_distance);
                        distanceCollection.push_back(distance);

                        /*
                        if (distance.bt_distance.m_distance < 0)
                        {
                            std::cout << "COLLISION" << std::endl;
                            std::cout << "Frame A:" << currentBody.name_collision_body << std::endl;
                            std::cout << "Frame B:" << collisionBody.name_collision_body << std::endl;
                            std::cout << "Distance:" << distance.bt_distance.m_distance << std::endl;
                        }
                        */
                    }
                }
            }
            // Find minimum distance
            pickMinimumDistance(distanceCollection,min_distances);
        }
    }
    // Calculate the reaction forces
    calculateReactionForce(min_distances,reaction_forces,ca_param_.self_collision);
}

void CollisionAvoidance::environmentCollision(std::vector<Distance> &min_distances, std::vector<ReactionForce> &reaction_forces)
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
            btVector3 bbx_dim;
            //btVector3 min_cb = btVector3(0,0,0);
            //btVector3 max_cb = btVector3(0,0,0);

            //findOuterPoints(collisionBody, min_cb, max_cb);

            /*
            xmin_bbx = min_cb[0];
            ymin_bbx = min_cb[1];
            zmin_bbx = min_cb[2];

            xmax_bbx = max_cb[0];
            ymax_bbx = max_cb[1];
            zmax_bbx = max_cb[2];
            */

            // Find largest dimension of the collision body
            double l = collisionBody.collision_shape.dimensions.x;
            if (l < collisionBody.collision_shape.dimensions.y) {
                l = collisionBody.collision_shape.dimensions.y;
            }
            if (l < collisionBody.collision_shape.dimensions.z) {
                l = collisionBody.collision_shape.dimensions.z;
            }

            btVector3 cb_im = btVector3(l+2*ca_param_.environment_collision.d_threshold,
                                        l+2*ca_param_.environment_collision.d_threshold,
                                        l+2*ca_param_.environment_collision.d_threshold);

            bbx_dim = collisionBody.bt_transform.getIdentity() * cb_im;


            xmin_bbx = collisionBody.bt_transform.getOrigin()[0] - bbx_dim[0];
            ymin_bbx = collisionBody.bt_transform.getOrigin()[1] - bbx_dim[1];
            zmin_bbx = collisionBody.bt_transform.getOrigin()[2] - bbx_dim[2];

            xmax_bbx = collisionBody.bt_transform.getOrigin()[0] + bbx_dim[0];
            ymax_bbx = collisionBody.bt_transform.getOrigin()[1] + bbx_dim[1];
            zmax_bbx = collisionBody.bt_transform.getOrigin()[2] + bbx_dim[2];


            /*
            // Check whether the BBX is inside the OctoMap, otherwise take OctoMap dimensions
            if (xmin_octomap > min_cb[0]) {
                xmin_bbx = xmin_octomap;
            } else {
                xmin_bbx = min_cb[0];
            }
            if (ymin_octomap > min_cb[1]) {
                ymin_bbx = ymin_octomap;
            } else {
                ymin_bbx = min_cb[1];
            }
            if (zmin_octomap > min_cb[2]) {
                zmin_bbx = xmin_octomap;
            } else {
                zmin_bbx = min_cb[2];
            }

            if (xmax_octomap < max_cb[0]) {
                xmax_bbx = xmax_octomap;
            } else {
                xmax_bbx = max_cb[0];
            }
            if (ymax_octomap < max_cb[1]) {
                ymax_bbx = ymax_octomap;
            } else {
                ymax_bbx = max_cb[1];
            }
            if (zmax_octomap < max_cb[2]) {
                zmax_bbx = zmax_octomap;
            } else {
                zmax_bbx = max_cb[2];
            }
            */


            // Set bounding box using its minimum and maximum coordinate
            octomath::Vector3 min = octomath::Vector3(xmin_bbx,
                                                      ymin_bbx,
                                                      zmin_bbx);

            octomath::Vector3 max = octomath::Vector3(xmax_bbx,
                                                      ymax_bbx,
                                                      zmax_bbx);


            //if (collisionBody.name_collision_body == "GripperRight")
            //{
                // Store bbx min max in vector for visualization
                min_.push_back(min);
                max_.push_back(max);
            //}


            // Find and store all occupied voxels
            std::vector<Voxel> in_range_voxels;
            for(octomap::OcTree::leaf_bbx_iterator it = octomap_->begin_leafs_bbx(min,max), end=octomap_->end_leafs_bbx(); it!= end; ++it)
            {
                Voxel vox;
                if (octomap_->isNodeOccupied(*it))
                {
                    vox.center_point.pose.position.x = it.getX();
                    vox.center_point.pose.position.y = it.getY();
                    vox.center_point.pose.position.z = it.getZ();
                    vox.center_point.pose.orientation.x = 0;
                    vox.center_point.pose.orientation.y = 0;
                    vox.center_point.pose.orientation.z = 0;
                    vox.center_point.pose.orientation.w = 1;

                    //std::cout << "voxel center position: " << vox.center_point.pose.position.x << " " << vox.center_point.pose.position.y << " " << vox.center_point.pose.position.z << std::endl;

                    // Get Voxel size
                    vox.size_voxel = octomap_->getNodeSize(it.getDepth());

                    in_range_voxels.push_back(vox);
                }
            }
            //std::cout << "in_range_voxels: " << in_range_voxels.size() << std::endl;



            std::vector<Distance> distanceCollection;
            for (std::vector<Voxel>::iterator itrVox = in_range_voxels.begin(); itrVox != in_range_voxels.end(); ++itrVox)
            {
                Voxel vox = *itrVox;
                RobotState::CollisionBody envBody;

                // Construct a Bullet Convex Shape of every Voxel within range
                envBody.bt_shape = new btBoxShape(btVector3(0.5*vox.size_voxel,0.5*vox.size_voxel,0.5*vox.size_voxel));
                setTransform(envBody.bt_transform, vox.center_point, no_fix_);

                distance.frame_id = collisionBody.fix_pose.header.frame_id;
                distanceCalculation(*collisionBody.bt_shape,*envBody.bt_shape,collisionBody.bt_transform,envBody.bt_transform,distance.bt_distance);
                distanceCollection.push_back(distance);

                //std::cout << "Distance:" << distance.bt_distance.m_distance << std::endl;
                /*
                if (distance.bt_distance.m_distance < 0)
                {
                    std::cout << "COLLISION" << std::endl;
                    std::cout << "Frame A:" << currentBody.name_collision_body << std::endl;
                    std::cout << "Frame B:" << collisionBody.name_collision_body << std::endl;
                    std::cout << "Distance:" << distance.bt_distance.m_distance << std::endl;
                }
                */
            }
            pickMinimumDistance(distanceCollection,min_distances);
        }
    }
    calculateReactionForce(min_distances,reaction_forces,ca_param_.environment_collision);
}

void CollisionAvoidance::getposeRPY(geometry_msgs::PoseStamped& pose, Eigen::Vector3d& RPY)
{
    tf::Quaternion Q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(pose.pose.orientation, Q);
    tf::Matrix3x3(Q).getRPY(roll, pitch, yaw);
    RPY(0) = roll;
    RPY(1) = pitch;
    RPY(2) = yaw;
}

void CollisionAvoidance::calculateWrenches(std::vector<ReactionForce> &reaction_forces, std::vector<Wrench> &wrenches_out)
{
    for (std::vector<ReactionForce>::iterator itrRF = reaction_forces.begin(); itrRF != reaction_forces.end(); ++itrRF)
    {
        ReactionForce &RF = *itrRF;
        Wrench W;
        geometry_msgs::PoseStamped p0;
        Eigen::VectorXd wrench_calc(6);
        wrench_calc.setZero();
        double dpx, dpy, dpz, fx, fy, fz;

        /*
        std::map<std::string, geometry_msgs::PoseStamped>::iterator itrFK = robot_state_.fk_poses_.find(RF.frame_id);
        geometry_msgs::PoseStamped p0 = (*itrFK).second;
        */

        for (std::vector< std::vector<RobotState::CollisionBody> >::iterator itrGroups = robot_state_->robot_.groups.begin(); itrGroups != robot_state_->robot_.groups.end(); ++itrGroups)
        {
            std::vector<RobotState::CollisionBody> &group = *itrGroups;
            for (std::vector<RobotState::CollisionBody>::iterator itrBodies = group.begin(); itrBodies != group.end(); ++itrBodies)
            {
                RobotState::CollisionBody &collisionBody = *itrBodies;
                if (collisionBody.fix_pose.header.frame_id == RF.frame_id )
                {
                    p0 = collisionBody.fk_pose;
                }
            }
        }

        dpx = RF.pointOnA.getX() - p0.pose.position.x;
        dpy = RF.pointOnA.getY() - p0.pose.position.y;
        dpz = RF.pointOnA.getZ() - p0.pose.position.z;

        fx = RF.amplitude*RF.direction.getX();
        fy = RF.amplitude*RF.direction.getY();
        fz = RF.amplitude*RF.direction.getZ();

        wrench_calc[0] = fx;
        wrench_calc[1] = fy;
        wrench_calc[2] = fz;
        wrench_calc[3] = fz*dpy - fy*dpz;
        wrench_calc[4] = fx*dpz - fz*dpx;
        wrench_calc[5] = fy*dpx - fx*dpy;

        W.frame_id = RF.frame_id;
        W.wrench = wrench_calc;

        wrenches_out.push_back(W);

        /*
        std::cout << "----------------------------------------------" << std::endl;
        std::cout << "Frame ID = " << W.frame_id << std::endl;
        std::cout << "dp = " << dpx << " " << dpy << " " << dpz << std::endl;
        std::cout << "f = " << W.wrench[0] << " " << W.wrench[1] << " " << W.wrench[2] << std::endl;
        std::cout << "T = " << W.wrench[3] << " " << W.wrench[4] << " " << W.wrench[5] << std::endl;
        */
    }
}

void CollisionAvoidance::visualize(std::vector<Distance> &min_distances) const
{
    int id = 0;
    for (std::vector< std::vector<RobotState::CollisionBody> >::const_iterator itrGroups = robot_state_->robot_.groups.begin(); itrGroups != robot_state_->robot_.groups.end(); ++itrGroups)
    {
        std::vector<RobotState::CollisionBody> group = *itrGroups;
        for (std::vector<RobotState::CollisionBody>::iterator itrsBodies = group.begin(); itrsBodies != group.end(); ++itrsBodies)
        {
            RobotState::CollisionBody &collisionBody = *itrsBodies;
            visualizeCollisionModel(collisionBody,id++);
        }
    }

    for (std::vector<Distance>::const_iterator itrdmin = min_distances.begin(); itrdmin != min_distances.end(); ++itrdmin)
    {
        Distance dmin = *itrdmin;
        visualizeReactionForce(dmin,id++);
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
    for (std::vector< std::vector<RobotState::CollisionBody> >::iterator it = robot_state_->robot_.groups.begin(); it != robot_state_->robot_.groups.end(); ++it)
    {
        std::vector<RobotState::CollisionBody> &group = *it;

        for (std::vector<RobotState::CollisionBody>::iterator it = group.begin(); it != group.end(); ++it)
        {
            RobotState::CollisionBody &collisionBody = *it;

            setTransform(collisionBody.bt_transform, collisionBody.fk_pose, collisionBody.fix_pose);
        }
    }
}

void CollisionAvoidance::setTransform(btTransform& transform_out, geometry_msgs::PoseStamped& fkPose, geometry_msgs::PoseStamped& fixPose)
{
    //cout << "FKPOSE: x = " << fkPose.pose.position.x << "y = " << fkPose.pose.position.y << "z = " << fkPose.pose.position.z << endl;
    btTransform fkTransform;
    btTransform fixTransform;
    fkTransform.setOrigin(btVector3(fkPose.pose.position.x,
                                    fkPose.pose.position.y,
                                    fkPose.pose.position.z));
    fkTransform.setRotation(btQuaternion(fkPose.pose.orientation.x,
                                         fkPose.pose.orientation.y,
                                         fkPose.pose.orientation.z,
                                         fkPose.pose.orientation.w));


    fixTransform.setOrigin(btVector3(fixPose.pose.position.x,
                                     fixPose.pose.position.y,
                                     fixPose.pose.position.z));
    fixTransform.setRotation(btQuaternion(fixPose.pose.orientation.x,
                                          fixPose.pose.orientation.y,
                                          fixPose.pose.orientation.z,
                                          fixPose.pose.orientation.w));
    transform_out.mult(fkTransform,fixTransform);


    /*
    std::cout << "--------------------------------------" << std::endl;
    std::cout << "frame_id = " << fixPose.header.frame_id << std::endl;
    std::cout << "frame_id = " << p.header.frame_id << std::endl;
    std::cout << "dx = " << fkPose.pose.position.x - p.pose.position.x << std::endl;
    std::cout << "dy = " << fkPose.pose.position.y - p.pose.position.y << std::endl;
    std::cout << "dz = " << fkPose.pose.position.z - p.pose.position.z << std::endl;
    std::cout << "dX = " << fkPose.pose.orientation.x - p.pose.orientation.x << std::endl;
    std::cout << "dY = " << fkPose.pose.orientation.y - p.pose.orientation.y << std::endl;
    std::cout << "dZ = " << fkPose.pose.orientation.z - p.pose.orientation.z << std::endl;
    std::cout << "dW = " << fkPose.pose.orientation.w - p.pose.orientation.w << std::endl;
    */
}


void CollisionAvoidance::distanceCalculation(btConvexShape& shapeA, btConvexShape& shapeB, btTransform& transformA, btTransform& transformB, btPointCollector &distance_out)
{
    btConvexPenetrationDepthSolver*	depthSolver = new btMinkowskiPenetrationDepthSolver;
    btSimplexSolverInterface* simplexSolver = new btVoronoiSimplexSolver;
    btGjkPairDetector convexConvex(&shapeA, &shapeB, simplexSolver, depthSolver);
    btGjkPairDetector::ClosestPointInput input;
    input.m_transformA = transformA;
    input.m_transformB = transformB;
    convexConvex.getClosestPoints(input, distance_out, 0);
}

void CollisionAvoidance::visualizeCollisionModel(RobotState::CollisionBody collisionBody,int id) const
{
    const btTransform& transform = collisionBody.bt_transform;
    std::string type = collisionBody.collision_shape.shape_type;
    string frame_id = collisionBody.fk_pose.header.frame_id;

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

        pub_marker_.publish(marker_array);
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

        pub_marker_.publish(marker_array);
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

        pub_marker_.publish(marker_array);
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

        pub_marker_.publish(marker_array);
    }

    else if (type == "Cone")
    {
        modelviz.type = visualization_msgs::Marker::MESH_RESOURCE;
        modelviz.header.frame_id = frame_id;
        modelviz.header.stamp = ros::Time::now();
        modelviz.mesh_resource = "package://amigo_whole_body_controller/data/cone.dae";
        modelviz.id = id;

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

        pub_marker_.publish(marker_array);
    }
}

void CollisionAvoidance::bulletTest()
{
    btConvexShape* A = new btSphereShape(1);
    btConvexShape* B = new btSphereShape(1);
    //btConvexShape* B = new btConeShapeZ(5-.04,1);
    btTransform At;
    btTransform Bt;
    btPointCollector d;

    At.setOrigin(btVector3(0,0,0));
    At.setRotation(btQuaternion(0,0,0,1));

    Bt.setOrigin(btVector3(2,0,0));
    Bt.setRotation(btQuaternion(0,0,0,1));

    distanceCalculation(*A,*B,At,Bt,d);

    std::cout << "distance = " << d.m_distance << std::endl;
    std::cout << "point on B = " << d.m_pointInWorld.getX() << " " << d.m_pointInWorld.getY() << " " << d.m_pointInWorld.getZ() << std::endl;
    std::cout << "normal on B = " << d.m_normalOnBInWorld.getX() << " " << d.m_normalOnBInWorld.getY() << " " << d.m_normalOnBInWorld.getZ() << std::endl;

}

void CollisionAvoidance::pickMinimumDistance(std::vector<Distance> &calculatedDistances, std::vector<Distance> &minimumDistances)
{
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

void CollisionAvoidance::calculateReactionForce(std::vector<Distance> &minimumDistances, std::vector<ReactionForce> &reactionForces, collisionAvoidanceParameters::Parameters &param)
{
    ReactionForce F;
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
            reactionForces.push_back(F);
        }
    }
}

void CollisionAvoidance::outputWrenches(std::vector<Wrench> &wrenches)
{
    for (std::vector<Wrench>::iterator itrW = wrenches.begin(); itrW != wrenches.end(); ++itrW)
    {
        Wrench W = *itrW;
        robot_state_->tree_.addCartesianWrench(W.frame_id,W.wrench);
    }
}

void CollisionAvoidance::visualizeReactionForce(Distance &d_min,int id) const
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
    RFviz.points.push_back(pA);

    pB.x = d_min.bt_distance.m_pointInWorld.getX() + d_min.bt_distance.m_distance * d_min.bt_distance.m_normalOnBInWorld.getX();
    pB.y = d_min.bt_distance.m_pointInWorld.getY() + d_min.bt_distance.m_distance * d_min.bt_distance.m_normalOnBInWorld.getY();
    pB.z = d_min.bt_distance.m_pointInWorld.getZ() + d_min.bt_distance.m_distance * d_min.bt_distance.m_normalOnBInWorld.getZ();
    RFviz.points.push_back(pB);

    if (d_min.bt_distance.m_distance <= ca_param_.self_collision.d_threshold )
    {     RFviz.color.a = 1;
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

    pub_marker_.publish(marker_array);

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

    pub_marker_.publish(marker_array);

}

void CollisionAvoidance::setOctoMap(const octomap_msgs::OctomapBinary& octomap_msg)
{
    // Only OctomapBinary is provided as a message in Fuerte
    // In Groovy a general Octomap message will be provided, so for now we'll have to do with the Binary

    octomap_ = octomap_msgs::binaryMsgDataToMap(octomap_msg.data);
}

void CollisionAvoidance::findOuterPoints(RobotState::CollisionBody& collisionBody, btVector3 min, btVector3 max)
{
    // Calculate all outer points
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
    std::cout << collisionBody.name_collision_body << std::endl;
    std::cout << "min: " << min[0] << " " << min[1] << " " << min[2] << std::endl;
    std::cout << "max: " << max[0] << " " << max[1] << " " << max[2] << std::endl;
}
