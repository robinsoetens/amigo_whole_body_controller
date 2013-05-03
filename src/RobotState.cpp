#include "RobotState.h"
#include "MotionObjective.h"

RobotState::RobotState() {

}

RobotState::~RobotState() {

}

void RobotState::collectFKSolutions(std::vector<Chain*> &chains,
                                    std::map<std::string, geometry_msgs::PoseStamped> &fk_poses)
{
    fk_poses.clear();
    geometry_msgs::PoseStamped fk_pose;
    std::pair<std::string, geometry_msgs::PoseStamped> fk_pair;

    fk_pose.header.frame_id = "base_link";
    fk_pose.pose.position.x = 0;
    fk_pose.pose.position.y = 0;
    fk_pose.pose.position.z = 0.055;
    fk_pose.pose.orientation.x = 0;
    fk_pose.pose.orientation.y = 0;
    fk_pose.pose.orientation.z = 0;
    fk_pose.pose.orientation.w = 1;

    fk_pair.first = "base_link";
    fk_pair.second = fk_pose;

    fk_poses["base_link"] = fk_pose;

    uint chainNr = 0;
    for (std::vector<Chain*>::iterator itrChain = chains.begin(); itrChain != chains.end(); ++itrChain)
    {
        Chain* &chain = *itrChain;

        for(unsigned int itr = 0; itr < chain->kdl_chain_.getNrOfSegments(); ++itr)
        {
            bool has_segement = false;
            for(std::map<std::string, geometry_msgs::PoseStamped>::iterator itrFK = fk_poses.begin(); itrFK != fk_poses.end(); ++itrFK)
            {
                std::pair<std::string, geometry_msgs::PoseStamped> FK = *itrFK;

                if( chain->kdl_chain_.getSegment(itr).getName() == FK.first )
                {
                    has_segement = true;
                }
            }

            if (has_segement == false)
            {
                geometry_msgs::PoseStamped fk_solution;
                KDL::Frame kdlFrame;
                KDL::ChainFkSolverPos_recursive* fk_solver;
                fk_solver = fk_solvers[chainNr];

                if (fk_solver->JntToCart(chain->joint_positions_,kdlFrame,itr) < 0 ) ROS_WARN("Problems with FK computation for the chain");
                kdlFrame.p.z(kdlFrame.p.z()+0.055);

                getFKsolution(kdlFrame, fk_solution);

                fk_poses[chain->kdl_chain_.getSegment(itr).getName()] = fk_solution;

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
            //std::cout << fk_solvers.size() << std::endl;            
        }
        chainNr++;
    }
}

void RobotState::separateChains(const std::vector<Chain*>& chains, RobotState &robot_state)
{
    //chain_left_ = 0;
    //chain_right_ = 0;
    for(std::vector<Chain*>::const_iterator it_chain = chains.begin(); it_chain != chains.end(); ++it_chain)
    {
        Chain* chain = *it_chain;

        if (chain->hasLink("grippoint_left")) {
            //std::cout << "Chain has end-effector frame: " << end_effector_frame_ << std::endl;
            robot_state.chain_left_ = chain;
        }
        else if (chain->hasLink("grippoint_right")) {
            //std::cout << "Chain has end-effector frame: " << end_effector_frame_ << std::endl;
            robot_state.chain_right_ = chain;
        }
    }

    //return (chain_left_ != 0 && chain_right_ != 0) ;
}

void RobotState::computeForwardKinematics(KDL::Frame& kdl_frame, const std::string& chain_side,int segmentNr, RobotState &robot_state)
{
    // Compute forward kinematics
    if (chain_side == "left") {
        if (fk_solver_left->JntToCart(robot_state.chain_left_->joint_positions_,kdl_frame,segmentNr) < 0 ) ROS_WARN("Problems with FK computation for the LEFT chain");
    }
    else if (chain_side == "right") {
        if (fk_solver_right->JntToCart(robot_state.chain_right_->joint_positions_,kdl_frame,segmentNr) < 0 ) ROS_WARN("Problems with FK computation for the RIGHT chain");
    }
    // Add 0.055 since base and base_link frame are not at exactly the same pose
    kdl_frame.p.z(kdl_frame.p.z()+0.055);
    //ROS_INFO("Return value = %i",ret);
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

void RobotState::calcPartialJacobian(std::string& frame_id, KDL::TreeJntToJacSolver* jac_solver_, Eigen::MatrixXd& jacobian)
{
    //jac_solver_->JntToJac(joint_positions_, jacobian);
}
