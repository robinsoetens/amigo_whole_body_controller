#include "ObstacleAvoidance.h"
#include <tf/transform_datatypes.h>

#include "visualization_msgs/MarkerArray.h"

using namespace std;

ObstacleAvoidance::ObstacleAvoidance(const string &end_effector_frame, tf::TransformListener *tf_listener)
    : listener_(*tf_listener), end_effector_frame_ (end_effector_frame) {
}

ObstacleAvoidance::~ObstacleAvoidance() {
    for(vector<Box*>::iterator it = boxes_.begin(); it != boxes_.end(); ++it) {
        delete *it;
    }
}

bool ObstacleAvoidance::initialize() {

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

bool ObstacleAvoidance::isActive() {
    return true;
}

void ObstacleAvoidance::apply(const RobotState &robotstate) {
    // Check which chain is active
        if (end_effector_frame_ == "grippoint_left") {
        end_effector_msg_ = robotstate.endEffectorPoseLeft;
        chain_ = robotstate.chain_left_;
    }
    else if (end_effector_frame_ == "grippoint_right") {
        end_effector_msg_ = robotstate.endEffectorPoseRight;
        chain_ = robotstate.chain_right_;
    }

    // Transform frame to map frame
    poseStampedMsgToTF(end_effector_msg_, tf_end_effector_pose_);

   try {
        listener_.transformPose("/map", tf_end_effector_pose_, tf_end_effector_pose_MAP_);
    } catch (tf::TransformException& e) {
        ROS_ERROR("CartesianImpedance: %s", e.what());
        return;
    }

    /*
    Eigen::Vector3d end_effector_RPY;
    getposeRPY(end_effector_msg_, end_effector_RPY);
    */

    //ROS_INFO("End effector pose: %f, %f, %f", end_effector_pose_MAP.getOrigin().getX(), end_effector_pose_MAP.getOrigin().getY(), end_effector_pose_MAP.getOrigin().getZ());

    // Calculate
    Eigen::Vector3d end_effector_pos(tf_end_effector_pose_MAP_.getOrigin().getX(),
                                     tf_end_effector_pose_MAP_.getOrigin().getY(),
                                     tf_end_effector_pose_MAP_.getOrigin().getZ());

    Eigen::Vector3d wrench;
    Eigen::VectorXd wrench_transformed(6);
    wrench.setZero();
    wrench_transformed.setZero();
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
            /*
                if (distance < 0.2) {
                    double x = distance;
                    weight = 10 / (x * x);
                } else if (distance < 0.3) {
                    weight = 0;//0.1 - (distance - 0.2) / 10;
                }
                */
            if (distance < 0.1) {
                weight = (1 - (distance / 0.1)) * 100;
            }
            for(unsigned int i = 0; i < 3; i++) {
                wrench(i) +=  diff_normalized(i) * weight;
            }

        }
    }

    transformWench(end_effector_msg_, wrench, wrench_transformed);


    visualize(end_effector_pos, wrench_transformed); // + (total_force / 10));
    chain_->addCartesianWrench(end_effector_frame_, wrench_transformed);

        // cout << F_task << endl;
}

void ObstacleAvoidance::visualize(const Eigen::Vector3d& end_effector_pos, const Eigen::VectorXd& wrench) const {

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



void ObstacleAvoidance::getposeRPY(geometry_msgs::PoseStamped& pose, Eigen::Vector3d& RPY) {

    tf::Quaternion Q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(pose.pose.orientation, Q);
    tf::Matrix3x3(Q).getRPY(roll, pitch, yaw);
    RPY(0) = roll;
    RPY(1) = pitch;
    RPY(2) = yaw;

}


void ObstacleAvoidance::transformWench(geometry_msgs::PoseStamped& pose, Eigen::Vector3d& wrench_in, Eigen::VectorXd &wrench_out) {

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
