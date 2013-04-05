#include "WholeBodyController.h"
#include "ChainParser.h"
#include <XmlRpcException.h>

using namespace std;

WholeBodyController::WholeBodyController(const double Ts)
{
    initialize(Ts);
}

WholeBodyController::~WholeBodyController() {
    ROS_INFO("Shutting down whole body controller");
}

bool WholeBodyController::initialize(const double Ts)
{
    ros::NodeHandle n("~");
    std::string ns = n.getNamespace();
    //ROS_INFO("Nodehandle %s",n.getNamespace().c_str());
    ROS_INFO("Initializing whole body controller");


    // ToDo: Parameterize
    Ts_ = Ts;

    ChainParser::parse(chains_, joint_name_to_index_, index_to_joint_name_, q_min_, q_max_);
    num_joints_ = joint_name_to_index_.size();

    cout << "Number of joints: " << num_joints_ << endl;

    q_current_.resize(num_joints_);
    for(unsigned int i = 0; i < q_current_.rows(); ++i)
    {
        q_current_(i) = 0;
    }

    // Read parameters
    std::vector<double> JLA_gain(num_joints_);      // Gain for the joint limit avoidance
    std::vector<double> JLA_workspace(num_joints_); // Workspace: joint gets 'pushed' back when it's outside of this part of the center of the workspace

    std::vector<double> posture_q0(num_joints_);
    std::vector<double> posture_gain(num_joints_);

    std::vector<double> admittance_mass(num_joints_);
    std::vector<double> admittance_damping(num_joints_);
    for (std::map<std::string, unsigned int>::iterator iter = joint_name_to_index_.begin(); iter != joint_name_to_index_.end(); ++iter)
    {
        n.param<double> (ns+"/joint_limit_avoidance/gain/"+iter->first, JLA_gain[iter->second], 1.0);
        n.param<double> (ns+"/joint_limit_avoidance/workspace/"+iter->first, JLA_workspace[iter->second], 0.9);
        n.param<double> (ns+"/posture_control/home_position/"+iter->first, posture_q0[iter->second], 0);
        n.param<double> (ns+"/posture_control/gain/"+iter->first, posture_gain[iter->second], 1);
        n.param<double> (ns+"/admittance_control/mass/"+iter->first, admittance_mass[iter->second], 10);
        n.param<double> (ns+"/admittance_control/damping/"+iter->first, admittance_damping[iter->second], 10);
        ROS_INFO("Damping joint %s = %f",iter->first.c_str(),admittance_damping[iter->second]);
    }


    typedef std::map<std::string, XmlRpc::XmlRpcValue>::iterator XmlRpcIterator;
    try
    {
        // ROBOT
        XmlRpc::XmlRpcValue groups;
        n.getParam(ns+"/collision_model", groups);
        for(XmlRpcIterator itrGroups = groups.begin(); itrGroups != groups.end(); ++itrGroups)
        {
            std::vector< RobotState::CollisionBody > robot_state_group;

            // COLLISION GROUP
            XmlRpc::XmlRpcValue group = itrGroups->second;
            for(XmlRpcIterator itrBodies = group.begin(); itrBodies != group.end(); ++itrBodies)
            {
                // COLLISION BODY
                XmlRpc::XmlRpcValue collisionBody = itrBodies->second;
                //cout << collisionBody["name"] << endl;

                RobotState::CollisionBody robotstate_collision_body;
                robotstate_collision_body.fromXmlRpc(collisionBody);

                // add the collision bodies to the group
                robot_state_group.push_back(robotstate_collision_body);
            }

            // add group of collision bodies to the robot
            robot_state_.robot_.groups.push_back(robot_state_group);

        }

    } catch(XmlRpc::XmlRpcException& ex)
    {
        cout << ex.getMessage() << endl;
    }


    createFKsolvers(chains_);

    fk_solver_left = new KDL::ChainFkSolverPos_recursive(robot_state_.chain_left_->kdl_chain_);
    fk_solver_right = new KDL::ChainFkSolverPos_recursive(robot_state_.chain_right_->kdl_chain_);

    // Initialize admittance controller
    AdmitCont_.initialize(Ts_,q_min_, q_max_, admittance_mass, admittance_damping);

    // Initialize nullspace calculator
    // ToDo: Make this variable
    Eigen::MatrixXd A;
    /*A.resize(num_joints_,num_joints_);
    for (uint i = 0; i < num_joints_; i++) {
        for (uint j = 0; j< num_joints_; j++) {
            if (i==j) A(i,j) = 1;
            else A(i,j) = 0;
        }
    }*/

    A.setIdentity(num_joints_,num_joints_);

    ComputeNullspace_.initialize(num_joints_, A);
    N_.resize(num_joints_,num_joints_);

    // Initialize Joint Limit Avoidance
    for (uint i = 0; i < num_joints_; i++) ROS_INFO("JLA gain of joint %i is %f",i,JLA_gain[i]);
    JointLimitAvoidance_.initialize(q_min_, q_max_, JLA_gain, JLA_workspace);
    ROS_INFO("Joint limit avoidance initialized");

    // Initialize Posture Controller
    for (uint i = 0; i < num_joints_; i++) posture_gain[i] = 1;
    PostureControl_.initialize(q_min_, q_max_, posture_q0, posture_gain);
    ROS_INFO("Posture Control initialized");

    // Resize additional variables
    F_task_.resize(0);
    qdot_reference_.resize(num_joints_);
    q_reference_.resize(num_joints_);
    Jacobian_.resize(12,num_joints_);                // Jacobian is resized to zero rows, number of rows depends on number of tasks
    tau_.resize(num_joints_);
    tau_nullspace_.resize(num_joints_);
    F_task_.resize(12);

    ROS_INFO("Whole Body Controller Initialized");

    return true;

}

bool WholeBodyController::addMotionObjective(MotionObjective* motionobjective)
{
    if (!motionobjective->initialize())
    {
        return false;
    }
    motionobjectives_.push_back(motionobjective);
    return true;
}

void WholeBodyController::setMeasuredJointPosition(const std::string& joint_name, double pos)
{
    q_current_(joint_name_to_index_[joint_name]) = pos;
    //std::cout << joint_name << q_current_(joint_name_to_index_[joint_name]) <<  std::endl;
}

bool WholeBodyController::update(KDL::JntArray q_current, Eigen::VectorXd& q_reference, Eigen::VectorXd& qdot_reference)
{

    //ROS_INFO("WholeBodyController::update()");

    // Set some variables to zero
    tau_.setZero();
    tau_nullspace_.setZero();
    F_task_.setZero();

    // Update the torque output
    // Currently only one torque 'source', when multiple ones a smarter solution has to be found
    // Plan of attack: set tau to zero --> add in every update loop.
    // Note that nullspace solution should be included in various subproblems
    // Not sure if this is the right approach: summing the taus up and then doing nullspace projection may result in smoother transitions and is probably quicker

    //std::cout << "q_current_ = " << std::endl;
    //std::cout << q_current_(joint_name_to_index_[wrist_yaw_joint_left]).data << std::endl;

    for(std::vector<Chain*>::iterator it_chain = chains_.begin(); it_chain != chains_.end(); ++it_chain)
    {
        Chain* chain = *it_chain;
        chain->removeCartesianWrenches();
        chain->setMeasuredJointPositions(q_current_);
    }

    KDL::Frame kdlFrameEndEffectorLeft;
    KDL::Frame kdlFrameEndEffectorRight;
    computeForwardKinematics(kdlFrameEndEffectorLeft,"left",getNrOfSegment(robot_state_.chain_left_->kdl_chain_,"grippoint_left"));
    getFKsolution(kdlFrameEndEffectorLeft, robot_state_.poseGrippointLeft_);
    computeForwardKinematics(kdlFrameEndEffectorRight,"right",getNrOfSegment(robot_state_.chain_left_->kdl_chain_,"grippoint_right"));
    getFKsolution(kdlFrameEndEffectorRight, robot_state_.poseGrippointRight_);


    for (std::vector< std::vector<RobotState::CollisionBody> >::iterator it = robot_state_.robot_.groups.begin(); it != robot_state_.robot_.groups.end(); ++it)
    {
        std::vector<RobotState::CollisionBody> &group = *it;

        for (std::vector<RobotState::CollisionBody>::iterator it = group.begin(); it != group.end(); ++it)
        {
            RobotState::CollisionBody &collisionBody = *it;
            KDL::Frame kdlFrame;

            // TO-DO GET RID OF HARD CODED
            if (collisionBody.fix_pose.header.frame_id == "base_link")
            {
                //computeForwardKinematics(kdlFrame,collisionBody.chain_side, getNrOfSegment(robot_state_.chain_left_->kdl_chain_,collisionBody.fix_pose.header.frame_id));
                //getFKsolution(kdlFrame, collisionBody.fk_pose);
                collisionBody.fk_pose.header.frame_id = "base_link";
                collisionBody.fk_pose.pose.position.x = 0;
                collisionBody.fk_pose.pose.position.y = 0;
                collisionBody.fk_pose.pose.position.z = 0.055;
                collisionBody.fk_pose.pose.orientation.x = 0;
                collisionBody.fk_pose.pose.orientation.y = 0;
                collisionBody.fk_pose.pose.orientation.z = 0;
                collisionBody.fk_pose.pose.orientation.w = 1;
            }
            else
            {
                if (collisionBody.chain_side == "left")
                {
                    computeForwardKinematics(kdlFrame, collisionBody.chain_side, getNrOfSegment(robot_state_.chain_left_->kdl_chain_, collisionBody.fix_pose.header.frame_id));
                    getFKsolution(kdlFrame, collisionBody.fk_pose);
                }
                else if (collisionBody.chain_side == "right")
                {
                    computeForwardKinematics(kdlFrame, collisionBody.chain_side, getNrOfSegment(robot_state_.chain_right_->kdl_chain_, collisionBody.fix_pose.header.frame_id));
                    getFKsolution(kdlFrame, collisionBody.fk_pose);
                }
            }
        }
    }

    for(std::vector<MotionObjective*>::iterator it_motionobjective = motionobjectives_.begin(); it_motionobjective != motionobjectives_.end(); ++it_motionobjective)
    {

        MotionObjective* motionobjective = *it_motionobjective;
        //ROS_INFO("Motion Objective: %p", motionobjective);
        if (motionobjective->isActive()) {
            motionobjective->apply(robot_state_);
        }
    }

    Eigen::VectorXd all_wrenches;
    Eigen::MatrixXd jacobian_full(0, num_joints_);
    jacobian_full.setZero();

    for(unsigned int i_chain = 0; i_chain < chains_.size(); ++i_chain)
    {
        Chain* chain = chains_[i_chain];
        chain->fillCartesianWrench(all_wrenches);
        chain->fillJacobian(jacobian_full);

    }

    //cout << "jacobian = " << endl << jacobian_full << endl;
    //cout << "all_wrenches = " << endl << all_wrenches << endl;

    tau_ = jacobian_full.transpose() * all_wrenches;
    //for (uint i = 0; i < tau_.rows(); i++) ROS_INFO("Task torques (%i) = %f",i,tau_(i));

    ComputeNullspace_.update(jacobian_full, N_);
    //ROS_INFO("Nullspace updated");
    //cout << "nullspace = " << endl << N_ << endl;

    JointLimitAvoidance_.update(q_current_, tau_nullspace_);
    ///ROS_INFO("Joint limit avoidance updated");

    PostureControl_.update(q_current_, tau_nullspace_);
    ///ROS_INFO("Posture control updated");

    tau_ += N_ * tau_nullspace_;
    //for (uint i = 0; i < tau_.rows(); i++) ROS_INFO("Total torques (%i) = %f",i,tau_(i));
    //cout << "tau_ = " << endl << tau_ << endl;

    AdmitCont_.update(tau_, qdot_reference_, q_current_, q_reference_);

    //for (uint i = 0; i < qdot_reference_.rows(); i++) ROS_INFO("qd joint %i = %f",i,qdot_reference_(i));
    //for (uint i = 0; i < q_current_.rows(); i++) ROS_INFO("Position joint %i = %f",i,q_current_(i));
    //for (uint i = 0; i < q_reference_.rows(); i++) ROS_INFO("Joint %i = %f",i,q_reference_(i));

    q_current = q_current_;
    q_reference = q_reference_;
    qdot_reference = qdot_reference_;

    //ROS_INFO("WholeBodyController::update() - end");

    return true;

}

const Eigen::VectorXd& WholeBodyController::getJointReferences() const
{
    return q_reference_;
}

const std::vector<std::string>& WholeBodyController::getJointNames() const
{
    return index_to_joint_name_;
}


void WholeBodyController::createFKsolvers(const std::vector<Chain*>& chains)
{
    //chain_left_ = 0;
    //chain_right_ = 0;
    for(std::vector<Chain*>::const_iterator it_chain = chains.begin(); it_chain != chains.end(); ++it_chain)
    {
        Chain* chain = *it_chain;

        if (chain->hasLink("grippoint_left")) {
            //std::cout << "Chain has end-effector frame: " << end_effector_frame_ << std::endl;
            robot_state_.chain_left_ = chain;
        }
        else if (chain->hasLink("grippoint_right")) {
            //std::cout << "Chain has end-effector frame: " << end_effector_frame_ << std::endl;
            robot_state_.chain_right_ = chain;
        }
    }

    //return (chain_left_ != 0 && chain_right_ != 0) ;
}


void WholeBodyController::computeForwardKinematics(KDL::Frame& kdl_frame, const std::string& chain_side,int segmentNr)
{
    // Compute forward kinematics
    if (chain_side == "left") {
        if (fk_solver_left->JntToCart(robot_state_.chain_left_->joint_positions_,kdl_frame,segmentNr) < 0 ) ROS_WARN("Problems with FK computation for the LEFT chain");
    }
    else if (chain_side == "right") {
        if (fk_solver_right->JntToCart(robot_state_.chain_right_->joint_positions_,kdl_frame,segmentNr) < 0 ) ROS_WARN("Problems with FK computation for the RIGHT chain");
    }
    // Add 0.055 since base and base_link frame are not at exactly the same pose
    kdl_frame.p.z(kdl_frame.p.z()+0.055);
    ///ROS_INFO("Return value = %i",ret);
    //if (is_active_) ROS_INFO("end_effector_pose_.p = [%f\t%f\t%f]",end_effector_pose_.p.x(),end_effector_pose_.p.y(),end_effector_pose_.p.z());

}


void WholeBodyController::getFKsolution(KDL::Frame& FK_pose, geometry_msgs::PoseStamped& pose)
{

    // ToDo: get rid of hardcoding
    pose.header.frame_id = "/base_link";
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

double WholeBodyController::getCost()
{

    // Set to zero
    double current_cost = 0;

    // Tau due to Cartesian Impedance + Collision avoidance
    for (int i = 0; i < tau_.rows(); i++) current_cost += fabs(tau_(i));

    // Joint limit avoidance
    current_cost += JointLimitAvoidance_.getCost();

    // Posture control
    current_cost += PostureControl_.getCost();

    // Singularity avoidance

    return current_cost;

}

int WholeBodyController::getNrOfSegment(KDL::Chain kdl_chain_, const std::string& segment_name)
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
