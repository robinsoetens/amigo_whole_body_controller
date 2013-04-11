#include "CollisionAvoidance.h"
#include <tf/transform_datatypes.h>

#include "visualization_msgs/MarkerArray.h"

using namespace std;

CollisionAvoidance::CollisionAvoidance(const double Ts, const string &end_effector_frame, tf::TransformListener *tf_listener)
    : Ts_ (Ts), end_effector_frame_ (end_effector_frame), listener_(*tf_listener)
{
}

CollisionAvoidance::~CollisionAvoidance()
{
    for(vector<Box*>::iterator it = boxes_.begin(); it != boxes_.end(); ++it)
    {
        delete *it;
    }
}

bool CollisionAvoidance::initialize(RobotState &robotstate)
{
    robot_state_ = robotstate;

    ROS_INFO_STREAM("Initializing Obstacle Avoidance, " << end_effector_frame_);

    // Get node handle
    ros::NodeHandle n("~");
    std::string ns = ros::this_node::getName();

    pub_marker_ = n.advertise<visualization_msgs::MarkerArray>("/whole_body_controller/environment_markers/"+end_effector_frame_, 10);

    bool wait_for_transform = listener_.waitForTransform(end_effector_frame_, "/map", ros::Time::now(), ros::Duration(1.0));
    if (!wait_for_transform) ROS_WARN("Transform between %s and /map is not available", end_effector_frame_.c_str());

    // initialize environment
    boxes_.push_back(new Box(Eigen::Vector3d(-0.127, 0.730, 0), Eigen::Vector3d(1.151, 1.139, 0.7)));
    boxes_.push_back(new Box(Eigen::Vector3d( 1.032, -0.942, 0), Eigen::Vector3d(1.590, 0.140, 0.6)));

    initializeCollisionModel(robotstate);

    ROS_INFO_STREAM("Initialized Obstacle Avoidance" << end_effector_frame_);

    return true;
}

bool CollisionAvoidance::isActive()
{
    return true;
}

void CollisionAvoidance::apply(RobotState &robotstate)
{

    robot_state_ = robotstate;
    calculateTransform();

    // Check which chain is active
    if (end_effector_frame_ == "grippoint_left")
    {
        end_effector_msg_ = robot_state_.poseGrippointLeft_;
        chain_ = robot_state_.chain_left_;
        contactpoint_msg_ = robot_state_.poseGrippointRight_;
    }
    else if (end_effector_frame_ == "grippoint_right")
    {
        end_effector_msg_ = robot_state_.poseGrippointRight_;
        chain_ = robot_state_.chain_right_;
        contactpoint_msg_ = robot_state_.poseGrippointLeft_;
    }

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

    // Calculate the wrench as a result of self-collision avoidance
    Eigen::VectorXd wrench_self_collision(6);
    wrench_self_collision.setZero();
    selfCollision(end_effector_msg_, contactpoint_msg_,wrench_self_collision);

    //ROS_INFO("wrench_self_collision = %f, \t%f, \t%f", wrench_self_collision(0), wrench_self_collision(1), wrench_self_collision(2));


    // Calculate the wrench as a result of environment collision avoidance
    Eigen::VectorXd wrench_environment_collision(6);
    wrench_environment_collision.setZero();
    environmentCollision(tf_end_effector_pose_MAP_, wrench_environment_collision);

    // Calculate the total wrench
    Eigen::VectorXd wrench_total(6);
    wrench_total.setZero();
    wrench_total = wrench_self_collision + wrench_environment_collision;


    // Correct self-collision avoidance vizualization for base orientation
    geometry_msgs::PoseStamped wrench_self_collision_msg;
    tf::Stamped<tf::Pose> tf_wrench_self_collision;
    tf::Stamped<tf::Pose> tf_wrench_self_collision_MAP;
    Eigen::VectorXd wrench_self_collision_vizualization(6);
    wrench_self_collision_vizualization.setZero();
    wrench_self_collision_msg.header.frame_id = end_effector_frame_;
    wrench_self_collision_msg.pose.position.x = wrench_self_collision(0);
    wrench_self_collision_msg.pose.position.y = wrench_self_collision(1);
    wrench_self_collision_msg.pose.position.z = wrench_self_collision(2);
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    wrench_self_collision_msg.pose.orientation = orientation;
    tf::poseStampedMsgToTF(wrench_self_collision_msg, tf_wrench_self_collision);

    try
    {
        listener_.transformPose("/map", tf_wrench_self_collision, tf_wrench_self_collision_MAP);
    } catch (tf::TransformException& e)
    {
        ROS_ERROR("CartesianImpedance: %s", e.what());
        return;
    }

    wrench_self_collision_vizualization(0) = tf_wrench_self_collision_MAP.getOrigin().getX();
    wrench_self_collision_vizualization(1) = tf_wrench_self_collision_MAP.getOrigin().getY();
    wrench_self_collision_vizualization(2) = tf_wrench_self_collision_MAP.getOrigin().getZ();

    // Output
    visualize(tf_end_effector_pose_MAP_, wrench_environment_collision + wrench_self_collision_vizualization); // + (total_force / 10));
    chain_->addCartesianWrench(end_effector_frame_, wrench_total);
}



void CollisionAvoidance::selfCollision(geometry_msgs::PoseStamped end_effector,geometry_msgs::PoseStamped contactpoint, Eigen::VectorXd& wrench_self_collision)
{
    // Transform the poses to the KDL::Frame format
    KDL::Frame end_effector_kdl_frame;
    KDL::Frame contactpoint_kdl_frame;
    Eigen::Vector3d end_effector_RPY;
    Eigen::Vector3d contactpoint_RPY;
    stampedPoseToKDLframe(end_effector, end_effector_kdl_frame, end_effector_RPY);
    stampedPoseToKDLframe(contactpoint, contactpoint_kdl_frame, contactpoint_RPY);

    collision_groups_.clear();
    active_group_.clear();
    for (std::vector< std::vector<RobotState::CollisionBody> >::iterator itrGroups = robot_state_.robot_.groups.begin(); itrGroups != robot_state_.robot_.groups.end(); ++itrGroups)
    {

        std::vector<RobotState::CollisionBody> &group = *itrGroups;

        bool group_check = false;
        for (std::vector<RobotState::CollisionBody>::iterator itrBodies = group.begin(); itrBodies != group.end(); ++itrBodies)
        {

            RobotState::CollisionBody &collisionBody = *itrBodies;
            //std::cout << collisionBody.bt_shape->getName() << std::endl;
            if ( collisionBody.fix_pose.header.frame_id == end_effector_frame_)
            {
                group_check = true;
            }
        }

        if ( group_check == true )
        {
            active_group_ = group;
        }
        else if ( group_check == false )
        {
            // IF NOT FOUND collision_group
            collision_groups_.push_back(group);
        }
    }


    //std::cout << "size active_group_ = " << active_group_.size() << std::endl;
    distances_.clear();
    for (std::vector<RobotState::CollisionBody>::iterator itrActiveBodies = active_group_.begin(); itrActiveBodies != active_group_.end(); ++itrActiveBodies)
    {
        RobotState::CollisionBody &currentBody = *itrActiveBodies;
        //std::cout << "size collision_groups_ = " << collision_groups_.size() << std::endl;
        for (std::vector<std::vector<RobotState::CollisionBody> >::iterator itrCollisionGroup = collision_groups_.begin(); itrCollisionGroup != collision_groups_.end(); ++itrCollisionGroup)
        {
            std::vector<RobotState::CollisionBody> &collisionGroup = *itrCollisionGroup;
            //std::cout << "size collisionGroup = " << collisionGroup.size() << std::endl;
            for (std::vector<RobotState::CollisionBody>::iterator itrCollisionBodies = collisionGroup.begin(); itrCollisionBodies != collisionGroup.end(); ++itrCollisionBodies)
            {
                RobotState::CollisionBody &collisionBody = *itrCollisionBodies;
                RobotState::Distance distance;
                distance.frame_id = currentBody.fix_pose.header.frame_id;
                //std::cout << currentBody.bt_shape->getName() << std::endl;
                //std::cout << "Self-Collision A" << std::endl;
                //std::cout << *currentBody.bt_shape->getName() << std::endl;
                distanceCalculation(*currentBody.bt_shape,*collisionBody.bt_shape,currentBody.bt_transform,collisionBody.bt_transform,distance.bt_distance);
                distances_.push_back(distance);

                //bulletTest();

                std::cout << "frame_id = " << distance.frame_id << std::endl;
                std::cout << "distance = " << distance.bt_distance.m_distance << std::endl;
                std::cout << "point on B = " << distance.bt_distance.m_pointInWorld.getX() << " " << distance.bt_distance.m_pointInWorld.getY() << " " << distance.bt_distance.m_pointInWorld.getZ() << std::endl;
                std::cout << "normal on B = " << distance.bt_distance.m_normalOnBInWorld.getX() << " " << distance.bt_distance.m_normalOnBInWorld.getY() << " " << distance.bt_distance.m_normalOnBInWorld.getZ() << std::endl;

            }
        }
    }


    // Calculate the difference between the two frames
    Eigen::Vector3d diff;
    diff.setZero();
    KDL::Twist diff_fk = KDL::diff(contactpoint_kdl_frame, end_effector_kdl_frame, Ts_);
    diff(0) = diff_fk.vel.x();
    diff(1) = diff_fk.vel.y();
    diff(2) = diff_fk.vel.z();

    //ROS_INFO("error pose = %f,\t%f,\t%f", diff_fk.vel.x(), diff_fk.vel.y(), diff_fk.vel.z());
    double threshold_self_collision = 10; // in centimeters
    double distance = diff.norm();
    Eigen::Vector3d diff_normalized = diff / distance;
    double weight = 0;
    if (distance < threshold_self_collision) {
        weight = (1 - (distance / threshold_self_collision)) * 100;
    }
    for(unsigned int i = 0; i < 3; i++) {
        wrench_self_collision(i) +=  diff_normalized(i) * weight;
    }
    //ROS_INFO("distance = %f", distance);

}

void CollisionAvoidance::environmentCollision(tf::Stamped<tf::Pose>& tf_end_effector_pose_MAP, Eigen::VectorXd& wrench_out)
{

    Eigen::Vector3d wrench;
    wrench.setZero();
    Eigen::Vector3d end_effector_pos(tf_end_effector_pose_MAP.getOrigin().getX(),
                                     tf_end_effector_pose_MAP.getOrigin().getY(),
                                     tf_end_effector_pose_MAP.getOrigin().getZ());


    double threshold_environment_collision = 0.1; // in meters
    for(vector<Box*>::iterator it = boxes_.begin(); it != boxes_.end(); ++it)
    {
        const Box& box = **it;

        bool inside_obstacle = true;
        Eigen::Vector3d diff;
        for(unsigned int i = 0; i < 3; ++i)
        {
            if (end_effector_pos[i] < box.min_[i]) {
                diff[i] = end_effector_pos[i] - box.min_[i];
                inside_obstacle = false;
            } else if (end_effector_pos[i] > box.max_[i]) {
                diff[i] = end_effector_pos[i] - box.max_[i];
                inside_obstacle = false;
            } else {
                diff[i] = 0;
            }
        }

        if (inside_obstacle) {
            ROS_ERROR("Inside obstacle!");
        } else {
            double distance = diff.norm();
            Eigen::Vector3d diff_normalized = diff / distance;
            double weight = 0;
            if (distance < threshold_environment_collision)
            {
                weight = (1 - (distance / threshold_environment_collision)) * 100;
            }
            for(unsigned int i = 0; i < 3; i++) {
                wrench(i) +=  diff_normalized(i) * weight;
            }

        }
    }

    transformWench(end_effector_msg_, wrench, wrench_out);

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

void CollisionAvoidance::transformWench(geometry_msgs::PoseStamped& pose, Eigen::Vector3d& wrench_in, Eigen::VectorXd &wrench_out)
{
    Eigen::Vector3d RPY;
    RPY.setZero();
    getposeRPY(pose,RPY);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(wrench_in[0], wrench_in[1], wrench_in[2]));

    tf::Quaternion Q;
    Q.setRPY(RPY(0),RPY(1),RPY(2));
    transform.setRotation(Q);

    wrench_out[0] = transform.getOrigin().x();
    wrench_out[1] = transform.getOrigin().y();
    wrench_out[2] = transform.getOrigin().z();
}

void CollisionAvoidance::stampedPoseToKDLframe(geometry_msgs::PoseStamped& pose, KDL::Frame& frame, Eigen::Vector3d& RPY)
{
    if (pose.header.frame_id != "/base_link") ROS_WARN("FK computation can now only cope with base_link as input frame");

    // Position
    frame.p.x(pose.pose.position.x);
    frame.p.y(pose.pose.position.y);
    frame.p.z(pose.pose.position.z);

    getposeRPY(pose, RPY);
}

void CollisionAvoidance::visualize(const tf::Stamped<tf::Pose>& tf_end_effector_pose_MAP, const Eigen::VectorXd& wrench) const
{
    Eigen::Vector3d end_effector_pos(tf_end_effector_pose_MAP.getOrigin().getX(),
                                     tf_end_effector_pose_MAP.getOrigin().getY(),
                                     tf_end_effector_pose_MAP.getOrigin().getZ());

    Eigen::Vector3d marker_pos;
    for(unsigned int i = 0; i < 3; ++i) {
        marker_pos(i) = end_effector_pos(i) + wrench(i) / 100;
    }

    visualization_msgs::MarkerArray marker_array;

    int id = 0;
    for(vector<Box*>::const_iterator it = boxes_.begin(); it != boxes_.end(); ++it)
    {
        const Box& box = **it;

        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.id = id++;

        marker.scale.x = (box.max_[0] - box.min_[0]);
        marker.scale.y = (box.max_[1] - box.min_[1]);
        marker.scale.z = (box.max_[2] - box.min_[2]);

        marker.pose.position.x = box.min_[0] + marker.scale.x / 2;
        marker.pose.position.y = box.min_[1] + marker.scale.y / 2;
        marker.pose.position.z = box.min_[2] + marker.scale.z / 2;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        marker.color.a = 1;
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 1;

        marker_array.markers.push_back(marker);
    }

    visualization_msgs::Marker dir;
    dir.type = visualization_msgs::Marker::SPHERE;
    dir.header.frame_id = "/map";
    dir.header.stamp = ros::Time::now();
    dir.id = id++;

    dir.scale.x = 0.1;
    dir.scale.y = 0.1;
    dir.scale.z = 0.1;

    dir.pose.position.x = marker_pos[0];
    dir.pose.position.y = marker_pos[1];
    dir.pose.position.z = marker_pos[2];
    dir.pose.orientation.x = 0;
    dir.pose.orientation.y = 0;
    dir.pose.orientation.z = 0;
    dir.pose.orientation.w = 1;

    dir.color.a = 1;
    dir.color.r = 1;
    dir.color.g = 0;
    dir.color.b = 0;

    marker_array.markers.push_back(dir);

    pub_marker_.publish(marker_array);

    for (std::vector< std::vector<RobotState::CollisionBody> >::const_iterator itrGroups = robot_state_.robot_.groups.begin(); itrGroups != robot_state_.robot_.groups.end(); ++itrGroups)
    {
        std::vector<RobotState::CollisionBody> group = *itrGroups;
        for (std::vector<RobotState::CollisionBody>::iterator itrsBodies = group.begin(); itrsBodies != group.end(); ++itrsBodies)
        {
            RobotState::CollisionBody &collisionBody = *itrsBodies;
            visualizeCollisionModel(collisionBody,id++);
        }
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
                collisionBody.bt_shape = new btConeShapeZ(x,z);
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
    for (std::vector< std::vector<RobotState::CollisionBody> >::iterator it = robot_state_.robot_.groups.begin(); it != robot_state_.robot_.groups.end(); ++it)
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

    /*
    std::cout << "frame_id = " << fixPose.header.frame_id << std::endl;
    std::cout << "x = " << fixPose.pose.position.x << std::endl;
    std::cout << "y = " << fixPose.pose.position.y << std::endl;
    std::cout << "z = " << fixPose.pose.position.z << std::endl;
    std::cout << "X = " << fixPose.pose.orientation.x << std::endl;
    std::cout << "Y = " << fixPose.pose.orientation.y << std::endl;
    std::cout << "Z = " << fixPose.pose.orientation.z << std::endl;
    std::cout << "W = " << fixPose.pose.orientation.w << std::endl;
    */

    transform_out.mult(fkTransform,fixTransform);
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
        modelviz.color.g = 1;
        modelviz.color.b = 0;

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
        modelviz.color.g = 1;
        modelviz.color.b = 0;

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
        modelviz.color.g = 1;
        modelviz.color.b = 0;

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
        modelviz.color.g = 1;
        modelviz.color.b = 0;

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

        modelviz.scale.x = 2*x;
        modelviz.scale.y = z;
        modelviz.scale.z = 2*y;

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
        modelviz.color.g = 1;
        modelviz.color.b = 0;

        marker_array.markers.push_back(modelviz);

        pub_marker_.publish(marker_array);
    }
}

void CollisionAvoidance::bulletTest()
{
    btConvexShape* A = new btBoxShape(btVector3(0.5,0.5,0.5));
    btConvexShape* B = new btSphereShape(1);
    btTransform At;
    btTransform Bt;
    btPointCollector d;

    At.setOrigin(btVector3(0,0,0));
    At.setRotation(btQuaternion(0,0,0,1));

    Bt.setOrigin(btVector3(5,0,0));
    Bt.setRotation(btQuaternion(0,0,0,1));

    /*
    btConvexPenetrationDepthSolver*	depthSolver = new btMinkowskiPenetrationDepthSolver;
    btSimplexSolverInterface* simplexSolver = new btVoronoiSimplexSolver;
    btGjkPairDetector convexConvex(A, B, simplexSolver, depthSolver);
    btGjkPairDetector::ClosestPointInput input;
    input.m_transformA = At;
    input.m_transformB = Bt;
    convexConvex.getClosestPoints(input, d, 0);
    */

    distanceCalculation(*A,*B,At,Bt,d);

    std::cout << "distance = " << d.m_distance << std::endl;
    std::cout << "point on B = " << d.m_pointInWorld.getX() << " " << d.m_pointInWorld.getY() << " " << d.m_pointInWorld.getZ() << std::endl;
    std::cout << "normal on B = " << d.m_normalOnBInWorld.getX() << " " << d.m_normalOnBInWorld.getY() << " " << d.m_normalOnBInWorld.getZ() << std::endl;

}
