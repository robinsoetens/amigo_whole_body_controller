#include "WholeBodyController.h"
#include "ChainParser.h"
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64MultiArray.h>

/// For tracing
#include <iostream>
#include <fstream>
#include <time.h>


using namespace std;

WholeBodyController::WholeBodyController(const double Ts)
{
    initialize(Ts);
}

WholeBodyController::~WholeBodyController() {
    ROS_INFO("Shutting down whole body controller");

    // ToDo: delete all motion objectives nicely?
}

bool WholeBodyController::initialize(const double Ts)
{
    ros::NodeHandle n("~");
    std::string ns = n.getNamespace();
    //ROS_INFO("Nodehandle %s",n.getNamespace().c_str());
    ROS_INFO("Initializing whole body controller");

    KDL::JntArray q_min, q_max;

    ChainParser::parse(robot_state_.chains_,robot_state_.tree_, joint_name_to_index_, index_to_joint_name_, q_min, q_max);
    num_joints_ = joint_name_to_index_.size();
    robot_state_.tree_.getJointNames(joint_name_to_index_,robot_state_.tree_.joint_name_to_index_);
    robot_state_.tree_.getTreeJointIndex(robot_state_.tree_.kdl_tree_, robot_state_.tree_.tree_joint_index_);

    //cout << "Number of joints: " << num_joints_ << endl;

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
        n.param<double> ("/whole_body_controller/joint_limit_avoidance/gain/"+iter->first, JLA_gain[iter->second], 1.0);
        n.param<double> ("/whole_body_controller/joint_limit_avoidance/workspace/"+iter->first, JLA_workspace[iter->second], 0.9);
        n.param<double> ("/whole_body_controller/posture_control/home_position/"+iter->first, posture_q0[iter->second], 0);
        n.param<double> ("/whole_body_controller/posture_control/gain/"+iter->first, posture_gain[iter->second], 1.0);
        n.param<double> ("/whole_body_controller/admittance_control/mass/"+iter->first, admittance_mass[iter->second], 10);
        n.param<double> ("/whole_body_controller/admittance_control/damping/"+iter->first, admittance_damping[iter->second], 10);
    }

    loadParameterFiles(robot_state_);

    // Construct the FK and Jacobian solvers
    robot_state_.fk_solver_ = new KDL::TreeFkSolverPos_recursive(robot_state_.tree_.kdl_tree_);
    robot_state_.tree_.jac_solver_ = new KDL::TreeJntToJacSolver(robot_state_.tree_.kdl_tree_);

    // Initialize admittance controller
    AdmitCont_.initialize(Ts,q_min, q_max, admittance_mass, admittance_damping);

    // Initialize nullspace calculator
    // ToDo: Make this variable
    Eigen::MatrixXd A;
    A.setIdentity(num_joints_,num_joints_);

    ComputeNullspace_.initialize(num_joints_, A);
    N_.resize(num_joints_,num_joints_);

    // Initialize Joint Limit Avoidance
    //for (uint i = 0; i < num_joints_; i++) ROS_INFO("JLA gain of joint %i is %f",i,JLA_gain[i]);
    JointLimitAvoidance_.initialize(q_min, q_max, JLA_gain, JLA_workspace);
    ROS_INFO("Joint limit avoidance initialized");

    // Initialize Posture Controller
    PostureControl_.initialize(q_min, q_max, posture_q0, posture_gain, joint_name_to_index_);
    ROS_INFO("Posture Control initialized");

    // Resize additional variables
    F_task_.resize(0);
    qdot_reference_.resize(num_joints_);
    q_reference_.resize(num_joints_);
    Jacobian_.resize(12,num_joints_);                // Jacobian is resized to zero rows, number of rows depends on number of tasks
    tau_.resize(num_joints_);
    tau_nullspace_.resize(num_joints_);
    F_task_.resize(12);

    /// Initialize tracer
    std::vector<std::string> column_names = index_to_joint_name_;
    column_names.push_back("joint_limit_cost");
    column_names.push_back("posture_cost");
    int buffersize;
    std::string foldername;
    n.param<int> ("/whole_body_controller/tracing_buffersize", buffersize, 0);
    n.param<std::string> ("/whole_body_controller/tracing_folder", foldername, "/tmp/");
    std::string filename = "wbc";
    std::string folderfilename = foldername + filename; // ToDo: make nice
    tracer_.Initialize(folderfilename, column_names, buffersize);

    ROS_INFO("Whole Body Controller Initialized");

    return true;

}

bool WholeBodyController::addMotionObjective(MotionObjective* motionobjective)
{
    if (!motionobjective->initialize(robot_state_))
    {
        return false;
    }
    motionobjectives_.push_back(motionobjective);
    return true;
}

bool WholeBodyController::removeMotionObjective(MotionObjective* motionobjective) {

    motionobjectives_.erase(std::remove(motionobjectives_.begin(), motionobjectives_.end(), motionobjective), motionobjectives_.end());

    return true;

}

void WholeBodyController::setMeasuredJointPosition(const std::string& joint_name, double pos)
{
    std::map<std::string,unsigned int>::iterator index_iter = joint_name_to_index_.find(joint_name);
    if (index_iter != joint_name_to_index_.end())
    {
        unsigned int index = index_iter->second;
        q_current_(index) = pos;

        //std::cout << joint_name <<"\t"<<q_current_(index) <<"\t"<<index<< std::endl;

        robot_state_.tree_.rearrangeJntArrayToTree(q_current_);
        //robot_state_.collectFKSolutions();
    }
    else
    {
        ROS_ERROR_ONCE("Joint %s is not listed",joint_name.c_str());
    }

}

double WholeBodyController::getJointPosition(const std::string& joint_name) const
{
    std::map<std::string,unsigned int>::const_iterator index_iter = joint_name_to_index_.find(joint_name);
    if (index_iter != joint_name_to_index_.end())
    {
        return q_current_(index_iter->second);
    } else {
        ROS_ERROR_ONCE("Joint %s is not listed",joint_name.c_str());
        return 0;
    }
}

bool WholeBodyController::setDesiredJointPosition(const std::string& joint_name, double reference)
{
    return PostureControl_.setJointTarget(joint_name, reference);
}

bool WholeBodyController::update(Eigen::VectorXd &q_reference, Eigen::VectorXd& qdot_reference)
{

    //ROS_INFO("WholeBodyController::update()");

    //cout << "Tree : " << robot_state_.tree_.kdl_tree_.getNrOfJoints() << endl;

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

    robot_state_.tree_.rearrangeJntArrayToTree(q_current_);
    robot_state_.tree_.removeCartesianWrenches();

    robot_state_.collectFKSolutions();

    /*
    std::cout << "==================================" << std::endl;
    for (std::map<std::string, KDL::Frame>::iterator itrFK = robot_state_.fk_poses_.begin(); itrFK != robot_state_.fk_poses_.end(); ++itrFK)
    {
        std::pair<std::string, KDL::Frame> FK = *itrFK;
        std::cout << "frame = " << FK.first << std::endl;
        std::cout << "X = " <<  FK.second.p.x() << std::endl;
        std::cout << "Y = " <<  FK.second.p.y() << std::endl;
        std::cout << "Z = " <<  FK.second.p.z() << std::endl;
    }
    */
    

    /// Apply the corresponding FK solution to the collision bodies
    for (std::vector< std::vector<RobotState::CollisionBody> >::iterator it = robot_state_.robot_.groups.begin(); it != robot_state_.robot_.groups.end(); ++it)
    {
        std::vector<RobotState::CollisionBody> &group = *it;

        for (std::vector<RobotState::CollisionBody>::iterator it = group.begin(); it != group.end(); ++it)
        {
            RobotState::CollisionBody &collisionBody = *it;
            std::map<std::string, KDL::Frame>::iterator itr = robot_state_.fk_poses_.find(collisionBody.frame_id);
            collisionBody.fk_pose = itr->second;
        }
    }

    /// Update motion objectives
    for(std::vector<MotionObjective*>::iterator it_motionobjective = motionobjectives_.begin(); it_motionobjective != motionobjectives_.end(); ++it_motionobjective)
    {
        MotionObjective* motionobjective = *it_motionobjective;
        //ROS_INFO("Motion Objective: %p", motionobjective);

        motionobjective->apply(robot_state_);

    }

    Eigen::VectorXd all_wrenches;
    Eigen::MatrixXd jacobian_full(0, num_joints_);
    jacobian_full.setZero();
    Eigen::MatrixXd jacobian_tree(0, num_joints_);
    jacobian_tree.setZero();

    //Eigen::MatrixXd jacobian(0, num_joints_);
    // ToDo: why can't we combine these functions?
    robot_state_.tree_.fillCartesianWrench(all_wrenches);
    robot_state_.tree_.fillJacobian(jacobian_tree);

    //cout << "jacobian = " << endl << jacobian_tree << endl;
    //cout << "all_wrenches = " << endl << all_wrenches << endl;
    tau_ = jacobian_tree.transpose() * all_wrenches;
    //for (uint i = 0; i < tau_.rows(); i++) ROS_INFO("Task torques (%i) = %f",i,tau_(i));

    ComputeNullspace_.update(jacobian_tree, N_);

    //ROS_INFO("Nullspace updated");
    //cout << "nullspace = " << endl << N_ << endl;

    JointLimitAvoidance_.update(q_current_,tau_nullspace_);
    ///ROS_INFO("Joint limit avoidance updated");

    PostureControl_.update(q_current_,tau_nullspace_);
    ///ROS_INFO("Posture control updated");

    tau_ += N_ * tau_nullspace_;
    //for (uint i = 0; i < tau_.rows(); i++) ROS_INFO("Total torques (%i) = %f",i,tau_(i));
    //cout << "tau_ = " << endl << tau_ << endl;

    AdmitCont_.update(tau_, qdot_reference_, q_current_, q_reference_);

    //for (uint i = 0; i < qdot_reference_.rows(); i++) ROS_INFO("qd joint %i = %f",i,qdot_reference_(i));
    //for (uint i = 0; i < q_current_.rows(); i++) ROS_INFO("Position joint %i = %f",i,q_current_(i));
    //for (uint i = 0; i < q_reference_.rows(); i++) ROS_INFO("Joint %i = %f",i,q_reference_(i));

    q_reference = q_reference_;
    qdot_reference = qdot_reference_;

    /// Only trace useful data (there are more motion objectives than the collision avoidance)...
    if (motionobjectives_.size() > 1) {
        tracer_.newLine();
        tracer_.collectTracing(1, q_current_.data);
        tracer_.collectTracing(num_joints_+1, JointLimitAvoidance_.getCost());
        tracer_.collectTracing(num_joints_+2, PostureControl_.getCost());
    }

    return true;

}

const Eigen::VectorXd& WholeBodyController::getJointReferences() const
{
    return q_reference_;
}

const Eigen::VectorXd& WholeBodyController::getJointTorques() const
{
    return tau_;
}

const std::vector<std::string>& WholeBodyController::getJointNames() const
{
    return index_to_joint_name_;
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

std::vector<MotionObjective*> WholeBodyController::getCartesianImpedances(const std::string& tip_frame, const std::string& root_frame) {

    std::vector<MotionObjective*> output;
    /// Loop through all motion objectives
    for (unsigned int i = 0; i < motionobjectives_.size(); i++) {
        /// Only account for motion objectives that are Cartesian impedances
        if (motionobjectives_[i]->type_ == "CartesianImpedance") {
            /// Add to the output vector if the tip frames are equal and if either the root frames are equal or the requested root frame is empty (in case all objectives of a certain tip frame are requested)
            if (motionobjectives_[i]->tip_frame_ == tip_frame && (motionobjectives_[i]->root_frame_ == root_frame || root_frame.empty())) {
                output.push_back(motionobjectives_[i]);
            }
        }
    }
    return output;
}

void WholeBodyController::loadParameterFiles(RobotState &robot_state)
{
    ros::NodeHandle n("~");
    std::string ns = ros::this_node::getName();
    XmlRpc::XmlRpcValue groups;
    typedef std::map<std::string, XmlRpc::XmlRpcValue>::iterator XmlRpcIterator;
    try
    {
        // ROBOT
        n.getParam("/whole_body_controller/collision_model", groups);
        for(XmlRpcIterator itrGroups = groups.begin(); itrGroups != groups.end(); ++itrGroups)
        {
            // COLLISION GROUP
            std::vector< RobotState::CollisionBody > robot_state_group;
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
            robot_state.robot_.groups.push_back(robot_state_group);
        }
        if (robot_state.robot_.groups.size() == 0)
        {
            ROS_WARN("No collision model loaded");
        }

        XmlRpc::XmlRpcValue exclusion_groups;
        n.getParam("/whole_body_controller/exlusions_collision_calculation", exclusion_groups);
        for(XmlRpcIterator itrExclGr = exclusion_groups.begin(); itrExclGr != exclusion_groups.end(); ++itrExclGr)
        {
            XmlRpc::XmlRpcValue ExclGroup = itrExclGr->second;
            for(XmlRpcIterator itrExcl = ExclGroup.begin(); itrExcl != ExclGroup.end(); ++itrExcl)
            {
                XmlRpc::XmlRpcValue Excl = itrExcl->second;
                RobotState::Exclusion exclusion;
                exclusion.fromXmlRpc(Excl);

                robot_state.exclusion_checks.checks.push_back(exclusion);
            }
        }

        if (robot_state.exclusion_checks.checks.size() == 0)
        {
            ROS_WARN("No exclusions from self-collision avoindance checks");
        }
        else if (robot_state.exclusion_checks.checks.size() > 0)
        {
            ROS_DEBUG("Exclusions from self-collision checks are: ");
            for (std::vector<RobotState::Exclusion>::iterator it = robot_state.exclusion_checks.checks.begin(); it != robot_state.exclusion_checks.checks.end(); ++it)
            {
                RobotState::Exclusion excl = *it;
                ROS_DEBUG("Name body A = %s", excl.name_body_A.c_str());
                ROS_DEBUG("Name body B = %s", excl.name_body_B.c_str());
            }
        }

    } catch(XmlRpc::XmlRpcException& ex)
    {
        cout << ex.getMessage() << endl;
    }

}

std::map<std::string, unsigned int> WholeBodyController::getJointNameToIndex()
{
    return joint_name_to_index_;
}
