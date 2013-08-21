#include "RobotState.h"
#include "MotionObjective.h"

RobotState::RobotState() {

}

RobotState::~RobotState() {

}

void RobotState::collectFKSolutions()
{
    fk_poses_.clear();
    fk_poses_["base_link"] = amcl_pose_;

    /// Fill in the vector with FK poses (why do we actually need to do this? We can 'find' the respective link in the tree and compute FK right away...)
    /// Iterate over tree
    KDL::SegmentMap treemap = tree_.kdl_tree_.getSegments();
    for (KDL::SegmentMap::iterator itrTree = treemap.begin(); itrTree != treemap.end(); ++itrTree)
    {
        /// Compute FK with respect to root frame of the tree
        KDL::Frame fk_pose_in_root;
        fk_solver_->JntToCart(tree_.q_tree_, fk_pose_in_root, itrTree->first);

        /// Convert to map frame
        KDL::Frame fk_pose_in_map;
        fk_pose_in_map = fk_pose_in_root * amcl_pose_;

        /// Put in map
        fk_poses_[itrTree->first] = fk_pose_in_map;

    }

}

void RobotState::KDLFrameToStampedPose(const KDL::Frame& FK_pose, geometry_msgs::PoseStamped& pose)
{
    // ToDo: get rid of hardcoding
    pose.header.frame_id = "map";
    // Position
    pose.pose.position.x = FK_pose.p.x();
    pose.pose.position.y = FK_pose.p.y();
    pose.pose.position.z = FK_pose.p.z();
    // Orientation
    FK_pose.M.GetQuaternion(pose.pose.orientation.x,
                            pose.pose.orientation.y,
                            pose.pose.orientation.z,
                            pose.pose.orientation.w);
}


int RobotState::getNrOfSegment(KDL::Chain kdl_chain_, const std::string& segment_name)
{
    int segmentNr = -1;
    for(unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); i++)
    {
        //cout << kdl_chain_.getSegment(i).getName().data() << endl;
        if (kdl_chain_.getSegment(i).getName().data() == segment_name)
        {
            //cout << "segment " << segment_name << " found" << endl;
            segmentNr = i+1;
            break;
        }
    }
    return segmentNr;
}

KDL::Frame RobotState::getFK(const std::string& tip_frame){

    /// Get the current FK pose as a geometry msgs
    std::map<std::string, KDL::Frame>::iterator itrFK = fk_poses_.find(tip_frame);
    KDL::Frame end_effector_pose = (*itrFK).second;

    return end_effector_pose;
}

geometry_msgs::PoseStamped RobotState::getFKPoseStamped(const std::string& tip_frame)
{
    /// Get the current FK pose as a geometry msgs
    std::map<std::string, KDL::Frame>::iterator itrFK = fk_poses_.find(tip_frame);
    KDL::Frame end_effector_pose = (*itrFK).second;

    geometry_msgs::PoseStamped pose_out;
    KDLFrameToStampedPose(end_effector_pose, pose_out);
    pose_out.header.frame_id = "/map"; //Robot state performs these computations in map
    return pose_out;

}

void RobotState::setAmclPose(const geometry_msgs::PoseWithCovarianceStamped& amcl_pose_msg)
{
    // Position
    amcl_pose_.p.x(amcl_pose_msg.pose.pose.position.x);
    amcl_pose_.p.y(amcl_pose_msg.pose.pose.position.y);
    amcl_pose_.p.z(amcl_pose_msg.pose.pose.position.z);

    // Orientation
    amcl_pose_.M.Quaternion(amcl_pose_msg.pose.pose.orientation.x,
                       amcl_pose_msg.pose.pose.orientation.y,
                       amcl_pose_msg.pose.pose.orientation.z,
                       amcl_pose_msg.pose.pose.orientation.w);

    fk_poses_["base_link"] = amcl_pose_;
}
