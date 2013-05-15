#include "CollisionAvoidance.h"
#include <tf/transform_datatypes.h>
#include <math.h>

#include "visualization_msgs/MarkerArray.h"

#include "profiling/Profiler.h"

using namespace std;

CollisionAvoidance::CollisionAvoidance(collisionAvoidanceParameters &parameters, const double Ts, tf::TransformListener *tf_listener)
    : ca_param_(parameters), Ts_ (Ts), listener_(*tf_listener)
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

    //bool wait_for_transform = listener_.waitForTransform(end_effector_frame_, "/map", ros::Time::now(), ros::Duration(1.0));
    //if (!wait_for_transform) ROS_WARN("Transform between %s and /map is not available", end_effector_frame_.c_str());

    initializeCollisionModel(robotstate);

    ROS_INFO_STREAM("Initialized Obstacle Avoidance");

    return true;
}

bool CollisionAvoidance::isActive()
{
    return true;
}

void CollisionAvoidance::apply(RobotState &robotstate)
{

    robot_state_ = &robotstate;
    calculateTransform();

    chain_left_ = robot_state_->chain_left_;
    chain_right_ = robot_state_->chain_right_;

    /*
    // Transform frame to map frame
    tf::poseStampedMsgToTF(end_effector_msg_, tf_end_effector_pose_);

    try
    {
        listener_.transformPose("/map", tf_end_effector_pose_, tf_end_effector_pose_MAP_);
    } catch (tf::TransformException& e)
    {
        ROS_ERROR("CollisionAvoidance: %s", e.what());
        return;
    }
    */

    // Calculate the wrenches as a result of self-collision avoidance
    std::vector<Distance> min_distances_total;
    std::vector<ReactionForce> reaction_forces_total;
    std::vector<Wrench> wrenches_total;
    min_distances_total.clear();
    reaction_forces_total.clear();
    wrenches_total.clear();
    selfCollision(min_distances_total, reaction_forces_total);
    calculateWrenches(reaction_forces_total, wrenches_total);

    // Output
    visualize(min_distances_total);
    outputWrenches(wrenches_total);

    /*
    std::cout << "==============================================" << std::endl;
    std::cout << "Size Reaction Force Vector = " << reaction_forces_total.size() << std::endl;
    std::cout << "==============================================" << std::endl;
    for (std::vector<ReactionForce>::iterator itrRF = reaction_forces_total.begin(); itrRF != reaction_forces_total.end(); ++itrRF)
    {
        ReactionForce &RF = *itrRF;
        std::cout << "----------------------------------------------" << std::endl;
        std::cout << "Frame ID = " << RF.frame_id << std::endl;
        std::cout << "point on A = " << RF.pointOnA.getX() << " " << RF.pointOnA.getY() << " " << RF.pointOnA.getZ() << std::endl;
        std::cout << "Direction of the force on A = " << RF.direction.getX() << " " << RF.direction.getY() << " " << RF.direction.getZ() << std::endl;
        std::cout << "Amplitude = " << RF.amplitude << std::endl;
    }
    */
}

void CollisionAvoidance::selfCollision(std::vector<Distance> &min_distances, std::vector<ReactionForce> &reaction_forces)
{
    active_groups_.clear();
    collision_groups_.clear();

    for (std::vector< std::vector<RobotState::CollisionBody> >::iterator itrGroups = robot_state_->robot_.groups.begin(); itrGroups != robot_state_->robot_.groups.end(); ++itrGroups)
    {
        std::vector<RobotState::CollisionBody> &group = *itrGroups;

        bool group_check = false;
        for (std::vector<RobotState::CollisionBody>::iterator itrBodies = group.begin(); itrBodies != group.end(); ++itrBodies)
        {
            RobotState::CollisionBody &collisionBody = *itrBodies;
            //std::cout << collisionBody.bt_shape->getName() << std::endl;
            if ( collisionBody.fix_pose.header.frame_id == chain_left_->kdl_chain_.getSegment(chain_left_->kdl_chain_.getNrOfSegments()-1).getName().data()
                 || collisionBody.fix_pose.header.frame_id == chain_right_->kdl_chain_.getSegment(chain_right_->kdl_chain_.getNrOfSegments()-1).getName().data())
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
                        //ThreadProfiler::Start("DistanceCalc");
                        distanceCalculation(*currentBody.bt_shape,*collisionBody.bt_shape,currentBody.bt_transform,collisionBody.bt_transform,distance.bt_distance);
                        distanceCollection.push_back(distance);
                        //ThreadProfiler::Stop("DistanceCalc");


                        if (distance.bt_distance.m_distance < 0)
                        {
                            std::cout << "COLLISION" << std::endl;
                            std::cout << "Frame A:" << currentBody.name_collision_body << std::endl;
                            std::cout << "Frame B:" << collisionBody.name_collision_body << std::endl;
                            std::cout << "Distance:" << distance.bt_distance.m_distance << std::endl;
                        }
                    }
                }
            }
            // Find minimum distance
            pickMinimumDistance(distanceCollection,min_distances);
        }
    }
    // Calculate the reaction forces
    calculateReactionForce(min_distances,reaction_forces);
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
        std::pair<std::string, geometry_msgs::PoseStamped> FK = *itrFK;
        geometry_msgs::PoseStamped p0 = FK.second;
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
                collisionBody.bt_shape = new btBoxShape(btVector3(0.5*x,0.5*y,0.5*z));
            }
            else if (type == "Sphere")
            {
                collisionBody.bt_shape = new btSphereShape(x);
            }
            else if (type == "Cone")
            {
                collisionBody.bt_shape = new btConeShapeZ(x-0.05,z);
            }
            else if (type == "CylinderY")
            {
                collisionBody.bt_shape = new btCylinderShape(btVector3(x,0.5*y,z));
            }
            else if (type == "CylinderZ")
            {
                collisionBody.bt_shape = new btCylinderShapeZ(btVector3(x,y,0.5*z));
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
            CollisionAvoidance::setTransform(collisionBody.bt_transform, collisionBody.fk_pose, collisionBody.fix_pose);
        }
    }
}

void CollisionAvoidance::setTransform(btTransform& transform_out, geometry_msgs::PoseStamped& fkPose, geometry_msgs::PoseStamped& fixPose)
{
    //cout << "FKPOSE: x = " << fkPose.pose.position.x << "y = " << fkPose.pose.position.y << "z = " << fkPose.pose.position.z << endl;
    btTransform fkTransform;
    btTransform fixTransform;

    // Get FK solution
    std::map<std::string, geometry_msgs::PoseStamped>::iterator itrFK = robot_state_->fk_poses_.find(fixPose.header.frame_id);
    std::pair<std::string, geometry_msgs::PoseStamped> FK = *itrFK;
    geometry_msgs::PoseStamped p = FK.second;
    /*
    fkTransform.setOrigin(btVector3(p.pose.position.x,
                                    p.pose.position.y,
                                    p.pose.position.z));
    fkTransform.setRotation(btQuaternion(p.pose.orientation.x,
                                         p.pose.orientation.y,
                                         p.pose.orientation.z,
                                         p.pose.orientation.w));
    */
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
    //std::cout << "A: " << transformA.getOrigin().getX() << " " << transformA.getOrigin().getY() << " " << transformA.getOrigin().getZ() << std::endl;
    //std::cout << "B: " << transformB.getOrigin().getX() << " " << transformB.getOrigin().getY() << " " << transformB.getOrigin().getZ() << std::endl;
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

        modelviz.scale.x = x;
        modelviz.scale.y = y;
        modelviz.scale.z = z;

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
        modelviz.scale.z = y;

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
        modelviz.scale.z = z;

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
        modelviz.scale.y = 1.5*z;
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

void CollisionAvoidance::calculateReactionForce(std::vector<Distance> &minimumDistances, std::vector<ReactionForce> &reactionForces)
{
    ReactionForce F;
    for (std::vector<Distance>::iterator itrMinDist = minimumDistances.begin(); itrMinDist != minimumDistances.end(); ++itrMinDist)
    {
        Distance &dmin = *itrMinDist;
        if (dmin.bt_distance.m_distance <= ca_param_.self_collision.d_threshold )
        {
            F.frame_id = dmin.frame_id;
            F.direction = dmin.bt_distance.m_normalOnBInWorld;
            F.pointOnA = dmin.bt_distance.m_pointInWorld + dmin.bt_distance.m_distance * dmin.bt_distance.m_normalOnBInWorld;
            F.amplitude = ca_param_.self_collision.f_max / pow(ca_param_.self_collision.d_threshold,ca_param_.self_collision.order) * pow((ca_param_.self_collision.d_threshold - dmin.bt_distance.m_distance),ca_param_.self_collision.order) ;

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

        //std::cout << "chain has link " << W.frame_id << ", wrench is added" << std::endl;

        /*
        if (chain_left_->hasLink(W.frame_id))
        {
            //Eigen::VectorXd wrench_calc(6);
            //wrench_calc.setZero();
            //chain_left_->addCartesianWrench("grippoint_left",wrench_calc);
            chain_left_->addCartesianWrench(W.frame_id,W.wrench);
        }
        else
        {
            //Eigen::VectorXd wrench_calc(6);
            //wrench_calc.setZero();
            //chain_right_->addCartesianWrench("grippoint_right",wrench_calc);
            chain_right_->addCartesianWrench(W.frame_id,W.wrench);
        }
        */

        /*
        for (std::vector<Chain*>::iterator itrChain = robot_state_.chains_.begin(); itrChain != robot_state_.chains_.end(); ++itrChain)
        {
            Chain* chain = *itrChain;

            if (chain->hasLink(W.frame_id))
            {
                chain->addCartesianWrench(W.frame_id,W.wrench);
                //std::cout << "chain has link " << W.frame_id << ", wrench is added" << std::endl;
                break;
            }
        }
        */

    }
}

void CollisionAvoidance::visualizeReactionForce(Distance &d_min,int id) const
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker RFviz;
    geometry_msgs::Point pA;
    geometry_msgs::Point pB;

    RFviz.type = visualization_msgs::Marker::ARROW;
    RFviz.header.frame_id = "/base_link";
    RFviz.header.stamp = ros::Time::now();
    RFviz.id = id++;

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
