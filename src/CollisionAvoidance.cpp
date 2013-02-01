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
    // Check which chain is active
    if (end_effector_frame_ == "grippoint_left") {
        end_effector_msg_ = robotstate.endEffectorPoseLeft;
        chain_ = robotstate.chain_left_;
        contactpoint_msg_ = robotstate.endEffectorPoseRight;
    }
    else if (end_effector_frame_ == "grippoint_right") {
        end_effector_msg_ = robotstate.endEffectorPoseRight;
        chain_ = robotstate.chain_right_;
        contactpoint_msg_ = robotstate.endEffectorPoseLeft;
    }

    // Transform frame to map frame
    tf::poseStampedMsgToTF(end_effector_msg_, tf_end_effector_pose_);

    try {
        listener_.transformPose("/map", tf_end_effector_pose_, tf_end_effector_pose_MAP_);
    } catch (tf::TransformException& e) {
        ROS_ERROR("CartesianImpedance: %s", e.what());
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
    geometry_msgs::PoseStamped base_pose;
    geometry_msgs::PoseStamped base_pose_MAP;
    tf::Stamped<tf::Pose> tf_base_pose;
    tf::Stamped<tf::Pose> tf_base_pose_MAP;
    Eigen::Vector3d wrench_self_collision_pos;
    base_pose.header.frame_id = "/base_link";
    base_pose.pose.position.x = 0;
    base_pose.pose.position.y = 0;
    base_pose.pose.position.z = 0;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    base_pose.pose.orientation = orientation;

    tf::poseStampedMsgToTF(base_pose, tf_base_pose);
    try {
        listener_.transformPose("/map", tf_base_pose, tf_base_pose_MAP);
    } catch (tf::TransformException& e) {
        ROS_ERROR("CartesianImpedance: %s", e.what());
        return;
    }
    tf::poseStampedTFToMsg(tf_base_pose_MAP, base_pose_MAP);
    //ROS_INFO("base orientation = %f, \t%f, \t%f, \t%f", base_pose_MAP.pose.orientation.w, base_pose_MAP.pose.orientation.x, base_pose_MAP.pose.orientation.y, base_pose_MAP.pose.orientation.z);


    wrench_self_collision_pos(0) = wrench_self_collision(0);
    wrench_self_collision_pos(1) = wrench_self_collision(1);
    wrench_self_collision_pos(2) = wrench_self_collision(2);
    Eigen::VectorXd wrench_self_collision_vizualization(6);
    wrench_self_collision_vizualization.setZero();

    transformWench(base_pose_MAP, wrench_self_collision_pos, wrench_self_collision_vizualization);
    //ROS_INFO("wrench_self_collision_pos  = %f,\t%f,\t%f", wrench_self_collision_pos(0), wrench_self_collision_pos(1), wrench_self_collision_pos(2));
    //ROS_INFO("wrench_self_collision_vizualization  = %f,\t%f,\t%f", wrench_self_collision_vizualization(0), wrench_self_collision_vizualization(1), wrench_self_collision_vizualization(2));

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
}
