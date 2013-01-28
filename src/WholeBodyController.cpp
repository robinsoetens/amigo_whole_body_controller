#include "WholeBodyController.h"
#include "ChainParser.h"

using namespace std;

WholeBodyController::WholeBodyController() {
    initialize();
}

WholeBodyController::~WholeBodyController() {
    ROS_INFO("Shutting down whole body controller");
}

bool WholeBodyController::initialize() {
    ros::NodeHandle n("~");
    std::string ns = n.getNamespace();
    //ROS_INFO("Nodehandle %s",n.getNamespace().c_str());
    ROS_INFO("Initializing whole body controller");

    // ToDo: Parameterize
    Ts = 0.1;

    ChainParser::parse(chains_, joint_name_to_index_, index_to_joint_name_, q_min_, q_max_);
    num_joints_ = joint_name_to_index_.size();

    cout << "Number of joints: " << num_joints_ << endl;

    q_current_.resize(num_joints_);
    for(unsigned int i = 0; i < q_current_.rows(); ++i) {
        q_current_(i) = 0;
    }

    // Read parameters
    std::vector<double> JLA_gain(num_joints_);      // Gain for the joint limit avoidance
    std::vector<double> JLA_workspace(num_joints_); // Workspace: joint gets 'pushed' back when it's outside of this part of the center of the workspace

    std::vector<double> posture_q0(num_joints_);
    std::vector<double> posture_gain(num_joints_);

    std::vector<double> admittance_mass(num_joints_);
    std::vector<double> admittance_damping(num_joints_);
    for (std::map<std::string, unsigned int>::iterator iter = joint_name_to_index_.begin(); iter != joint_name_to_index_.end(); ++iter) {
        n.param<double> (ns+"/joint_limit_avoidance/gain/"+iter->first, JLA_gain[iter->second], 1.0);
        n.param<double> (ns+"/joint_limit_avoidance/workspace/"+iter->first, JLA_workspace[iter->second], 0.9);
        n.param<double> (ns+"/posture_control/home_position/"+iter->first, posture_q0[iter->second], 0);
        n.param<double> (ns+"/posture_control/gain/"+iter->first, posture_gain[iter->second], 1);
        n.param<double> (ns+"/admittance_control/mass/"+iter->first, admittance_mass[iter->second], 10);
        n.param<double> (ns+"/admittance_control/damping/"+iter->first, admittance_damping[iter->second], 10);
        ROS_INFO("Damping joint %s = %f",iter->first.c_str(),admittance_damping[iter->second]);
    }

    createFKsolvers(chains_);

    fk_solver_left = new KDL::ChainFkSolverPos_recursive(robot_state_.chain_left_->kdl_chain_);
    fk_solver_right = new KDL::ChainFkSolverPos_recursive(robot_state_.chain_right_->kdl_chain_);

    // Initialize admittance controller
    AdmitCont_.initialize(q_min_, q_max_, admittance_mass, admittance_damping);

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

bool WholeBodyController::addMotionObjective(MotionObjective* motionobjective) {
    if (!motionobjective->initialize()) {
        return false;
    }
    motionobjectives_.push_back(motionobjective);
    return true;
}

void WholeBodyController::setMeasuredJointPosition(const std::string& joint_name, double pos) {
    q_current_(joint_name_to_index_[joint_name]) = pos;
    //std::cout << joint_name << q_current_(joint_name_to_index_[joint_name]) <<  std::endl;
}

bool WholeBodyController::update() {

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

    for(std::vector<Chain*>::iterator it_chain = chains_.begin(); it_chain != chains_.end(); ++it_chain) {
        Chain* chain = *it_chain;
        chain->removeCartesianWrenches();
        chain->setMeasuredJointPositions(q_current_);
    }

    KDL::Frame end_effector_pose_left;
    KDL::Frame end_effector_pose_right;
    computeForwardKinematics(end_effector_pose_left, "/grippoint_left");
    getFKsolution(end_effector_pose_left, robot_state_.endEffectorPoseLeft);
    computeForwardKinematics(end_effector_pose_right, "/grippoint_right");
    getFKsolution(end_effector_pose_right, robot_state_.endEffectorPoseRight);

    for(std::vector<MotionObjective*>::iterator it_motionobjective = motionobjectives_.begin(); it_motionobjective != motionobjectives_.end(); ++it_motionobjective) {

        MotionObjective* motionobjective = *it_motionobjective;
        //ROS_INFO("Motion Objective: %p", motionobjective);
        if (motionobjective->isActive()) {
            motionobjective->apply(robot_state_);
        }
    }

    Eigen::VectorXd all_wrenches;
    Eigen::MatrixXd jacobian_full(0, num_joints_);
    jacobian_full.setZero();

    for(unsigned int i_chain = 0; i_chain < chains_.size(); ++i_chain) {
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


    //ROS_INFO("WholeBodyController::update() - end");

    return true;

}

const Eigen::VectorXd& WholeBodyController::getJointReferences() const {
    return q_reference_;
}

const std::vector<std::string>& WholeBodyController::getJointNames() const {
    return index_to_joint_name_;
}

void WholeBodyController::computeForwardKinematics(KDL::Frame& FK_end_effector_pose,
                                                   const std::string& end_effector_frame) {

    // Compute forward kinematics
    if (end_effector_frame == "/grippoint_left") {
        if (fk_solver_left->JntToCart(robot_state_.chain_left_->joint_positions_, FK_end_effector_pose) < 0 ) ROS_WARN("Problems with FK computation for the LEFT chain");
    }
    else if (end_effector_frame == "/grippoint_right") {
        if (fk_solver_right->JntToCart(robot_state_.chain_right_->joint_positions_, FK_end_effector_pose) < 0 ) ROS_WARN("Problems with FK computation for the RIGHT chain");
    }
    // Add 0.055 since base and base_link frame are not at exactly the same pose
    FK_end_effector_pose.p.z(FK_end_effector_pose.p.z()+0.055);
    ///ROS_INFO("Return value = %i",ret);
    //if (is_active_) ROS_INFO("end_effector_pose_.p = [%f\t%f\t%f]",end_effector_pose_.p.x(),end_effector_pose_.p.y(),end_effector_pose_.p.z());

}

void WholeBodyController::getFKsolution(KDL::Frame& FK_end_effector_pose,
                                        geometry_msgs::PoseStamped& pose) {

    // ToDo: get rid of hardcoding
    pose.header.frame_id = "/base_link";
    // Position
    pose.pose.position.x = FK_end_effector_pose.p.x();
    pose.pose.position.y = FK_end_effector_pose.p.y();
    pose.pose.position.z = FK_end_effector_pose.p.z();
    // Orientation
    FK_end_effector_pose.M.GetQuaternion(pose.pose.orientation.x,
                                         pose.pose.orientation.y,
                                         pose.pose.orientation.z,
                                         pose.pose.orientation.w);

}

void WholeBodyController::createFKsolvers(const std::vector<Chain*>& chains) {
    //chain_left_ = 0;
    //chain_right_ = 0;
    for(std::vector<Chain*>::const_iterator it_chain = chains.begin(); it_chain != chains.end(); ++it_chain) {
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
