#include "RobotState.h"
#include "MotionObjective.h"

RobotState::RobotState() {

}

RobotState::~RobotState() {

}

void RobotState::collectFKSolutions(tf::TransformListener& listener)
{
    fk_poses_.clear();
    geometry_msgs::PoseStamped baselink_pose;

    // Get base_link frame using tf
    tf::StampedTransform tf_baselink;
    tf::StampedTransform tf_baselink_fix;
    //geometry_msgs::PoseStamped msg_baselink;
    try
    {
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/map","/base_link",now, ros::Duration(1.0));
        listener.lookupTransform("/map","/base_link",now,tf_baselink);
        //listener_.transformPose("/map", robot_pose, robot_pose_MAP);
    } catch (tf::TransformException& e)
    {
        ROS_ERROR("WholeBodyController: %s", e.what());
    }

    // Add 0.055 to z coordinate since base and base_link frame are not at exactly the same pose
    tf_baselink_fix.setOrigin(btVector3(tf_baselink.getOrigin().getX(),
                                        tf_baselink.getOrigin().getY(),
                                        tf_baselink.getOrigin().getZ()));
    tf_baselink_fix.setRotation(tf_baselink.getRotation());

    baselink_pose.header.frame_id = "map";
    baselink_pose.pose.position.x = tf_baselink_fix.getOrigin().getX();
    baselink_pose.pose.position.y = tf_baselink_fix.getOrigin().getY();
    baselink_pose.pose.position.z = tf_baselink_fix.getOrigin().getZ();
    baselink_pose.pose.orientation.x = tf_baselink_fix.getRotation().getX();
    baselink_pose.pose.orientation.y = tf_baselink_fix.getRotation().getY();
    baselink_pose.pose.orientation.z = tf_baselink_fix.getRotation().getZ();
    baselink_pose.pose.orientation.w = tf_baselink_fix.getRotation().getW();

    fk_poses_["base_link"] = baselink_pose;

    // ToDo:: Get rid of chains
    uint chainNr = 0;
    for (std::vector<Chain*>::iterator itrChain = chains_.begin(); itrChain != chains_.end(); ++itrChain)
    {
        Chain* &chain = *itrChain;

        for(unsigned int itr = 0; itr < chain->kdl_chain_.getNrOfSegments(); ++itr)
        {
            bool has_segement = false;
            for(std::map<std::string, geometry_msgs::PoseStamped>::iterator itrFK = fk_poses_.begin(); itrFK != fk_poses_.end(); ++itrFK)
            {
                if( chain->kdl_chain_.getSegment(itr).getName() == (*itrFK).first )
                {
                    has_segement = true;
                }
            }

            if (has_segement == false)
            {
                geometry_msgs::PoseStamped fk_pose,fk_solution;
                tf::Stamped<tf::Pose> tf_pose,tf_solution;
                KDL::Frame kdlFrame;

                if (fk_solver_->JntToCart(tree_.q_tree_, kdlFrame,chain->kdl_chain_.getSegment(itr).getName()) < 0 ) ROS_WARN("Problems with FK computation for the chain");

                // Transform FK solution to map frame using precomputed base_link solution
                getFKsolution(kdlFrame, fk_pose);
                tf::poseStampedMsgToTF(fk_pose,tf_pose);
                tf_solution.mult(tf_baselink_fix,tf_pose);
                tf::poseStampedTFToMsg(tf_solution,fk_solution);
                fk_solution.header.frame_id = "map";

                fk_poses_[chain->kdl_chain_.getSegment(itr).getName()] = fk_solution;

                /*
                std::cout << "frame = " << chain->kdl_chain_.getSegment(itr).getName() << std::endl;
                std::cout << "frame_id = " <<  fk_solution.header.frame_id << std::endl;
                std::cout << "x = " <<  fk_solution.pose.position.x << std::endl;
                std::cout << "y = " <<  fk_solution.pose.position.y << std::endl;
                std::cout << "z = " <<  fk_solution.pose.position.z << std::endl;
                std::cout << "X = " <<  fk_solution.pose.orientation.x << std::endl;
                std::cout << "Y = " <<  fk_solution.pose.orientation.y << std::endl;
                std::cout << "Z = " <<  fk_solution.pose.orientation.z << std::endl;
                std::cout << "W = " <<  fk_solution.pose.orientation.w << std::endl;
                */
            }

        }
        //std::cout << fk_solvers.size() << std::endl;
    }
    chainNr++;
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

void RobotState::getFKsolution(KDL::Frame& FK_pose, geometry_msgs::PoseStamped& pose)
{
    // ToDo: get rid of hardcoding
    pose.header.frame_id = "base_link";
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
