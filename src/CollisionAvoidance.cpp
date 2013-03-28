#include "CollisionAvoidance.h"
#include <tf/transform_datatypes.h>

#include "visualization_msgs/MarkerArray.h"

using namespace std;

CollisionAvoidance::CollisionAvoidance(const double Ts, const string &end_effector_frame, tf::TransformListener *tf_listener)
    : Ts_ (Ts), end_effector_frame_ (end_effector_frame), listener_(*tf_listener) {
}

CollisionAvoidance::~CollisionAvoidance() {
    for(vector<Box*>::iterator it = boxes_.begin(); it != boxes_.end(); ++it) {
        delete *it;
    }
}

bool CollisionAvoidance::initialize() {

    ROS_INFO_STREAM("Initializing Obstacle Avoidance, " << end_effector_frame_);

    // Get node handle
    ros::NodeHandle n("~");
    std::string ns = ros::this_node::getName();

    pub_marker_ = n.advertise<visualization_msgs::MarkerArray>("/whole_body_controller/environment_markers/"+end_effector_frame_, 10);

    bool wait_for_transform = listener_.waitForTransform(end_effector_frame_, "/map", ros::Time::now(), ros::Duration(1.0));
    if (!wait_for_transform) ROS_WARN("Transform between %s and /map is not available", end_effector_frame_.c_str());

    // initialize environment
    boxes_.push_back(new Box(Eigen::Vector3d(-0.127, 0.730, 0), Eigen::Vector3d(1.151, 1.139, 0.7)));
    boxes_.push_back(new Box(Eigen::Vector3d( 1.032, -0.942, 0), Eigen::Vector3d(1.590, 0.140, 0.5)));

    ROS_INFO_STREAM("Initialized Obstacle Avoidance" << end_effector_frame_);

    return true;
}

bool CollisionAvoidance::isActive() {
    return true;
}

void CollisionAvoidance::apply(const RobotState &robotstate) {

    robot_state_ = robotstate;

    // Check which chain is active
    if (end_effector_frame_ == "grippoint_left") {
        end_effector_msg_ = robot_state_.poseGrippointLeft_;
        chain_ = robot_state_.chain_left_;
        contactpoint_msg_ = robot_state_.poseGrippointRight_;
    }
    else if (end_effector_frame_ == "grippoint_right") {
        end_effector_msg_ = robot_state_.poseGrippointRight_;
        chain_ = robot_state_.chain_right_;
        contactpoint_msg_ = robot_state_.poseGrippointLeft_;
    }

    // Transform frame to map frame
    tf::poseStampedMsgToTF(end_effector_msg_, tf_end_effector_pose_);

    try {
        listener_.transformPose("/map", tf_end_effector_pose_, tf_end_effector_pose_MAP_);
    } catch (tf::TransformException& e) {
        ROS_ERROR("CollisionAvoidance: %s", e.what());
        return;
    }

    collisionModel();

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

    try {
        listener_.transformPose("/map", tf_wrench_self_collision, tf_wrench_self_collision_MAP);
    } catch (tf::TransformException& e) {
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



void CollisionAvoidance::selfCollision(geometry_msgs::PoseStamped end_effector,geometry_msgs::PoseStamped contactpoint, Eigen::VectorXd& wrench_self_collision) {

    // Transform the poses to the KDL::Frame format
    KDL::Frame end_effector_kdl_frame;
    KDL::Frame contactpoint_kdl_frame;
    Eigen::Vector3d end_effector_RPY;
    Eigen::Vector3d contactpoint_RPY;
    stampedPoseToKDLframe(end_effector, end_effector_kdl_frame, end_effector_RPY);
    stampedPoseToKDLframe(contactpoint, contactpoint_kdl_frame, contactpoint_RPY);


    int k = 0;
    btPointCollector btDistance;
    if (end_effector_frame_ == "grippoint_left") {
        for (uint i = 0; i < shapesLeftArm.size(); i++) {
            for (uint j = 0; j < shapesBody.size(); j++) {
                distanceCalculation(*shapesLeftArm[i],*shapesBody[j],tranformsLeftArm[i],tranformsBody[j],btDistance);
                distances.push_back(btDistance);
                k++;
            }

            for (uint j = 0; j < shapesRightArm.size(); j++) {
                distanceCalculation(*shapesLeftArm[i],*shapesRightArm[j],tranformsLeftArm[i],tranformsRightArm[j],btDistance);
                distances.push_back(btDistance);
                k++;
            }
        }
    }

    else if (end_effector_frame_ == "grippoint_right") {
        for (uint i = 0; i < shapesRightArm.size(); i++) {
            for (uint j = 0; j < shapesBody.size(); j++) {
                distanceCalculation(*shapesRightArm[i],*shapesBody[j],tranformsRightArm[i],tranformsBody[j],btDistance);
                distances.push_back(btDistance);
                k++;
            }

            for (uint j = 0; j < shapesLeftArm.size(); j++) {
                distanceCalculation(*shapesRightArm[i],*shapesLeftArm[j],tranformsRightArm[i],tranformsLeftArm[j],btDistance);
                distances.push_back(btDistance);
                k++;
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


void CollisionAvoidance::environmentCollision(tf::Stamped<tf::Pose>& tf_end_effector_pose_MAP, Eigen::VectorXd& wrench_out) {

    Eigen::Vector3d wrench;
    wrench.setZero();
    Eigen::Vector3d end_effector_pos(tf_end_effector_pose_MAP.getOrigin().getX(),
                                     tf_end_effector_pose_MAP.getOrigin().getY(),
                                     tf_end_effector_pose_MAP.getOrigin().getZ());


    double threshold_environment_collision = 0.1; // in meters
    for(vector<Box*>::iterator it = boxes_.begin(); it != boxes_.end(); ++it) {
        const Box& box = **it;

        bool inside_obstacle = true;
        Eigen::Vector3d diff;
        for(unsigned int i = 0; i < 3; ++i) {
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
            if (distance < threshold_environment_collision) {
                weight = (1 - (distance / threshold_environment_collision)) * 100;
            }
            for(unsigned int i = 0; i < 3; i++) {
                wrench(i) +=  diff_normalized(i) * weight;
            }

        }
    }

    transformWench(end_effector_msg_, wrench, wrench_out);

}


void CollisionAvoidance::getposeRPY(geometry_msgs::PoseStamped& pose, Eigen::Vector3d& RPY) {

    tf::Quaternion Q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(pose.pose.orientation, Q);
    tf::Matrix3x3(Q).getRPY(roll, pitch, yaw);
    RPY(0) = roll;
    RPY(1) = pitch;
    RPY(2) = yaw;

}


void CollisionAvoidance::transformWench(geometry_msgs::PoseStamped& pose, Eigen::Vector3d& wrench_in, Eigen::VectorXd &wrench_out) {

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


void CollisionAvoidance::stampedPoseToKDLframe(geometry_msgs::PoseStamped& pose, KDL::Frame& frame, Eigen::Vector3d& RPY) {

    if (pose.header.frame_id != "/base_link") ROS_WARN("FK computation can now only cope with base_link as input frame");

    // Position
    frame.p.x(pose.pose.position.x);
    frame.p.y(pose.pose.position.y);
    frame.p.z(pose.pose.position.z);

    getposeRPY(pose, RPY);

}

void CollisionAvoidance::visualize(const tf::Stamped<tf::Pose>& tf_end_effector_pose_MAP, const Eigen::VectorXd& wrench) const {

    Eigen::Vector3d end_effector_pos(tf_end_effector_pose_MAP.getOrigin().getX(),
                                     tf_end_effector_pose_MAP.getOrigin().getY(),
                                     tf_end_effector_pose_MAP.getOrigin().getZ());

    Eigen::Vector3d marker_pos;
    for(unsigned int i = 0; i < 3; ++i) {
        marker_pos(i) = end_effector_pos(i) + wrench(i) / 100;
    }

    visualization_msgs::MarkerArray marker_array;

    int id = 0;
    for(vector<Box*>::const_iterator it = boxes_.begin(); it != boxes_.end(); ++it) {
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

    visualizeCollisionModel(*btBaseBottom_, btTransformBaseBottom_,robot_state_.poseBase_.header.frame_id,id++,0.36,0.36,0.5*0.28);
    visualizeCollisionModel(*btBaseMiddle_, btTransformBaseMiddle_,robot_state_.poseBase_.header.frame_id,id++,0.2,0.28,0.19);
    visualizeCollisionModel(*btBaseTop_, btTransformBaseTop_,robot_state_.poseBase_.header.frame_id,id++,0.25,0.25,.5*0.9);
    visualizeCollisionModel(*btSliders_, btTransformSliders_,robot_state_.poseSliders_.header.frame_id,id++,0.7,0.23,0.15);
    visualizeCollisionModel(*btTorso_, btTransformTorso_,robot_state_.poseTorso_.header.frame_id,id++,0.18,0.18,0.18);
    visualizeCollisionModel(*btClavicles_, btTransformClavicles_,robot_state_.poseClavicles_.header.frame_id,id++,0.1,0.6,0.16);
    visualizeCollisionModel(*btUpperArmLeft_, btTransformUpperArmLeft_,robot_state_.poseUpperArmLeft_.header.frame_id,id++,0.07,0.07,0.35);
    visualizeCollisionModel(*btUpperArmRight_, btTransformUpperArmRight_,robot_state_.poseUpperArmRight_.header.frame_id,id++,0.07,0.07,0.35);
    visualizeCollisionModel(*btForeArmLeft_, btTransformForeArmLeft_,robot_state_.poseForeArmLeft_.header.frame_id,id++,0.05,0.05,0.32);
    visualizeCollisionModel(*btForeArmRight_, btTransformForeArmRight_,robot_state_.poseForeArmRight_.header.frame_id,id++,0.05,0.05,0.32);
    visualizeCollisionModel(*btGripperLeft_, btTransformGripperLeft_,robot_state_.poseGrippointLeft_.header.frame_id,id++,0.2,0.17,0.05);
    visualizeCollisionModel(*btGripperRight_, btTransformGripperRight_,robot_state_.poseGrippointRight_.header.frame_id,id++,0.2,0.17,0.05);

}

void CollisionAvoidance::collisionModel(){

    // Construction of the Primitive Shapes
    btBaseBottom_ = new btCylinderShapeZ(btVector3(0.36,0.36,0.5*0.28));
    btBaseMiddle_ = new btBoxShape(btVector3(0.2*0.5,0.5*0.28,0.5*0.19));
    btBaseTop_ = new btConeShapeZ(0.25,0.9);
    btSliders_ = new btBoxShape(btVector3(0.7*0.5,0.23*0.5,0.15*0.5)); //btBoxShape(btVector3(x*0.5,y*0.5,z*0.5))
    btTorso_ =  new btSphereShape(0.19); //btSphereShape(radius)
    btClavicles_ = new btBoxShape(btVector3(0.1*0.5,0.6*0.5,0.16*0.5));
    btUpperArmLeft_ = new btCylinderShape(btVector3(0.07,0.35*0.5,0.07)); //btCylinderShape(btVector3(radius,height*0.5,radius))
    btUpperArmRight_= new btCylinderShape(btVector3(0.07,0.35*0.5,0.07)); //centered around the origin, its central axis aligned with the Y axis
    btForeArmLeft_ = new btCylinderShape(btVector3(0.05,0.32*0.5,0.05));
    btForeArmRight_ = new btCylinderShape(btVector3(0.05,0.32*0.5,0.05));
    btGripperLeft_ = new btBoxShape(btVector3(0.2*0.5,0.17*0.5,0.05*0.5));
    btGripperRight_ = new btBoxShape(btVector3(0.2*0.5,0.17*0.5,0.05*0.5));
    //btHead_ = btBoxShape(btVector3(0.255*0.5,0.13*0.5,0.11*0.5)); //Head Cover, for Kinect use btBoxShape(btVector3(0.28*0.5,0.06*0.5,0.035*0.5))

    geometry_msgs::PoseStamped fixPoseBaseBottom;
    fixPoseBaseBottom.pose.position.x = 0.0;
    fixPoseBaseBottom.pose.position.y = 0.0;
    fixPoseBaseBottom.pose.position.z = 0.5*0.14;
    fixPoseBaseBottom.pose.orientation.x = 0.0;
    fixPoseBaseBottom.pose.orientation.y = 0.0;
    fixPoseBaseBottom.pose.orientation.z = 0.0;
    fixPoseBaseBottom.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped fixPoseBaseMiddle;
    fixPoseBaseMiddle.pose.position.x = 0.25;
    fixPoseBaseMiddle.pose.position.y = 0.0;
    fixPoseBaseMiddle.pose.position.z = 0.14+0.5*0.19;
    fixPoseBaseMiddle.pose.orientation.x = 0.0;
    fixPoseBaseMiddle.pose.orientation.y = 0.0;
    fixPoseBaseMiddle.pose.orientation.z = 0.0;
    fixPoseBaseMiddle.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped fixPoseBaseTop;
    fixPoseBaseTop.pose.position.x = 0.02;
    fixPoseBaseTop.pose.position.y = 0.0;
    fixPoseBaseTop.pose.position.z = 0.1+0.5*0.9;
    fixPoseBaseTop.pose.orientation.x = 0.0;
    fixPoseBaseTop.pose.orientation.y = 0.0;
    fixPoseBaseTop.pose.orientation.z = 0.0;
    fixPoseBaseTop.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped fixPoseSliders;
    fixPoseSliders.pose.position.x = -0.2;
    fixPoseSliders.pose.position.y = 0.0;
    fixPoseSliders.pose.position.z = -0.04;
    fixPoseSliders.pose.orientation.x = 0.0;
    fixPoseSliders.pose.orientation.y = 0.0;
    fixPoseSliders.pose.orientation.z = 0.0;
    fixPoseSliders.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped fixPoseTorso;
    fixPoseTorso.pose.position.x = 0.01;
    fixPoseTorso.pose.position.y = 0.0;
    fixPoseTorso.pose.position.z = 0.005;
    fixPoseTorso.pose.orientation.x = 0.0;
    fixPoseTorso.pose.orientation.y = 0.0;
    fixPoseTorso.pose.orientation.z = 0.0;
    fixPoseTorso.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped fixPoseClavicles;
    fixPoseClavicles.pose.position.x = 0.05;
    fixPoseClavicles.pose.position.y = 0.0;
    fixPoseClavicles.pose.position.z = 0.0;
    fixPoseClavicles.pose.orientation.x = 0.0;
    fixPoseClavicles.pose.orientation.y = 0.0;
    fixPoseClavicles.pose.orientation.z = 0.0;
    fixPoseClavicles.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped fixPoseUpperArmLeft;
    fixPoseUpperArmLeft.pose.position.x = -0.01;
    fixPoseUpperArmLeft.pose.position.y = -0.35*0.5;
    fixPoseUpperArmLeft.pose.position.z = 0.0;
    fixPoseUpperArmLeft.pose.orientation.x = 0.0;
    fixPoseUpperArmLeft.pose.orientation.y = 0.0;
    fixPoseUpperArmLeft.pose.orientation.z = 0.0;
    fixPoseUpperArmLeft.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped fixPoseUpperArmRight;
    fixPoseUpperArmRight.pose.position.x = -0.01;
    fixPoseUpperArmRight.pose.position.y = -0.35*0.5;
    fixPoseUpperArmRight.pose.position.z = 0.0;
    fixPoseUpperArmRight.pose.orientation.x = 0.0;
    fixPoseUpperArmRight.pose.orientation.y = 0.0;
    fixPoseUpperArmRight.pose.orientation.z = 0.0;
    fixPoseUpperArmRight.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped fixPoseForeArmLeft;
    fixPoseForeArmLeft.pose.position.x = 0.0;
    fixPoseForeArmLeft.pose.position.y = -0.32*0.5;
    fixPoseForeArmLeft.pose.position.z = 0.0;
    fixPoseForeArmLeft.pose.orientation.x = 0.0;
    fixPoseForeArmLeft.pose.orientation.y = 0.0;
    fixPoseForeArmLeft.pose.orientation.z = 0.0;
    fixPoseForeArmLeft.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped fixPoseForeArmRight;
    fixPoseForeArmRight.pose.position.x = 0.0;
    fixPoseForeArmRight.pose.position.y = -0.32*0.5;
    fixPoseForeArmRight.pose.position.z = 0.0;
    fixPoseForeArmRight.pose.orientation.x = 0.0;
    fixPoseForeArmRight.pose.orientation.y = 0.0;
    fixPoseForeArmRight.pose.orientation.z = 0.0;
    fixPoseForeArmRight.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped fixPoseGrippointLeft;
    fixPoseGrippointLeft.pose.position.x = -0.02;
    fixPoseGrippointLeft.pose.position.y = 0.0;
    fixPoseGrippointLeft.pose.position.z = 0.0;
    fixPoseGrippointLeft.pose.orientation.x = 0.0;
    fixPoseGrippointLeft.pose.orientation.y = 0.0;
    fixPoseGrippointLeft.pose.orientation.z = 0.0;
    fixPoseGrippointLeft.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped fixPoseGrippointRight;
    fixPoseGrippointRight.pose.position.x = -0.02;
    fixPoseGrippointRight.pose.position.y = 0.0;
    fixPoseGrippointRight.pose.position.z = 0.0;
    fixPoseGrippointRight.pose.orientation.x = 0.0;
    fixPoseGrippointRight.pose.orientation.y = 0.0;
    fixPoseGrippointRight.pose.orientation.z = 0.0;
    fixPoseGrippointRight.pose.orientation.w = 1.0;
    //geometry_msgs::PoseStamped fixPoseHead;

    // Positioning and Orientation of the Primitive Shapes
    setTransform(btTransformBaseBottom_, robot_state_.poseBase_,fixPoseBaseBottom);
    setTransform(btTransformBaseMiddle_, robot_state_.poseBase_,fixPoseBaseMiddle);
    setTransform(btTransformBaseTop_, robot_state_.poseBase_,fixPoseBaseTop);
    setTransform(btTransformSliders_, robot_state_.poseSliders_,fixPoseSliders);
    setTransform(btTransformTorso_, robot_state_.poseTorso_,fixPoseTorso);
    setTransform(btTransformClavicles_, robot_state_.poseClavicles_,fixPoseClavicles);
    setTransform(btTransformUpperArmLeft_, robot_state_.poseUpperArmLeft_,fixPoseUpperArmLeft);
    setTransform(btTransformUpperArmRight_, robot_state_.poseUpperArmRight_,fixPoseUpperArmRight);
    setTransform(btTransformForeArmLeft_, robot_state_.poseForeArmLeft_,fixPoseForeArmLeft);
    setTransform(btTransformForeArmRight_, robot_state_.poseForeArmRight_,fixPoseForeArmRight);
    setTransform(btTransformGripperLeft_, robot_state_.poseGrippointLeft_,fixPoseGrippointLeft);
    setTransform(btTransformGripperRight_, robot_state_.poseGrippointRight_,fixPoseGrippointRight);
    //setTransform(btTransformHead_, robot_state_.poseHead_,fixPoseHead);


    shapesBody.push_back(btBaseBottom_);
    shapesBody.push_back(btBaseMiddle_);
    shapesBody.push_back(btBaseTop_);
    shapesBody.push_back(btSliders_);
    shapesBody.push_back(btTorso_);
    shapesBody.push_back(btClavicles_);
    shapesBody.push_back(btUpperArmLeft_);
    shapesBody.push_back(btUpperArmRight_);

    tranformsBody.push_back(btTransformBaseBottom_);
    tranformsBody.push_back(btTransformBaseMiddle_);
    tranformsBody.push_back(btTransformBaseTop_);
    tranformsBody.push_back(btTransformSliders_);
    tranformsBody.push_back(btTransformTorso_);
    tranformsBody.push_back(btTransformClavicles_);
    tranformsBody.push_back(btTransformUpperArmLeft_);
    tranformsBody.push_back(btTransformUpperArmRight_);

    shapesLeftArm.push_back(btForeArmLeft_);
    shapesLeftArm.push_back(btGripperLeft_);

    tranformsLeftArm.push_back(btTransformForeArmLeft_);
    tranformsLeftArm.push_back(btTransformGripperLeft_);

    shapesRightArm.push_back(btForeArmRight_);
    shapesRightArm.push_back(btGripperRight_);

    tranformsRightArm.push_back(btTransformForeArmRight_);
    tranformsRightArm.push_back(btTransformGripperRight_);

}


void CollisionAvoidance::setTransform(btTransform& transform_out, geometry_msgs::PoseStamped& fkPose, geometry_msgs::PoseStamped& fixPose) {
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

}


void CollisionAvoidance::distanceCalculation(btConvexShape& shapeA,btConvexShape& shapeB,btTransform& transformA,btTransform& transformB,btPointCollector distance_out) {
    btConvexPenetrationDepthSolver*	depthSolver = new btMinkowskiPenetrationDepthSolver;
    btSimplexSolverInterface* simplexSolver = new btVoronoiSimplexSolver;

    btGjkPairDetector convexConvex(&shapeA, &shapeB, simplexSolver, depthSolver);
    btGjkPairDetector::ClosestPointInput input;
    input.m_transformA = transformA;
    input.m_transformB = transformB;
    convexConvex.getClosestPoints(input, distance_out, 0);
}

void CollisionAvoidance::visualizeCollisionModel(btConvexShape& btshape, const btTransform& transform, string frame_id, int id, double length, double width, double height) const {
    visualization_msgs::Marker modelviz;
    visualization_msgs::MarkerArray marker_array;
    std::string type = btshape.getName();

    if (type == "Box") {
        modelviz.type = visualization_msgs::Marker::CUBE;
        modelviz.header.frame_id = frame_id;
        modelviz.header.stamp = ros::Time::now();
        modelviz.id = id;

        modelviz.scale.x = length;
        modelviz.scale.y = width;
        modelviz.scale.z = height;

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

    else if (type == "SPHERE") {
        modelviz.type = visualization_msgs::Marker::SPHERE;
        modelviz.header.frame_id = frame_id;
        modelviz.header.stamp = ros::Time::now();
        modelviz.id = id;

        modelviz.scale.x = 2*length;
        modelviz.scale.y = 2*width;
        modelviz.scale.z = 2*height;

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

    else if (type == "CylinderY") {
        modelviz.type = visualization_msgs::Marker::CYLINDER;
        modelviz.header.frame_id = frame_id;
        modelviz.header.stamp = ros::Time::now();
        modelviz.id = id;

        modelviz.scale.x = 2*length;
        modelviz.scale.y = 2*width;
        modelviz.scale.z = height;

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

    else if (type == "CylinderZ") {
        modelviz.type = visualization_msgs::Marker::CYLINDER;
        modelviz.header.frame_id = frame_id;
        modelviz.header.stamp = ros::Time::now();
        modelviz.id = id;

        modelviz.scale.x = 2*length;
        modelviz.scale.y = 2*width;
        modelviz.scale.z = height;

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

    else if (type == "Cone") {
        modelviz.type = visualization_msgs::Marker::MESH_RESOURCE;
        modelviz.header.frame_id = frame_id;
        modelviz.header.stamp = ros::Time::now();
        modelviz.mesh_resource = "package://amigo_whole_body_controller/data/cone.dae";
        modelviz.id = id;

        modelviz.scale.x = 2*length;
        modelviz.scale.y = 2*height;
        modelviz.scale.z = 2*width;

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
