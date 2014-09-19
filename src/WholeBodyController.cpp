#include "WholeBodyController.h"

#include "ChainParser.h"
#include <ros/node_handle.h>
#include <ros/this_node.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64MultiArray.h>

/// For tracing
#include <iostream>
#include <fstream>
#include <time.h>



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

    if ( !ChainParser::parse(robot_state_.tree_, joint_name_to_index_, index_to_joint_name_, q_min, q_max) ) {
        return false;
    }

    num_joints_ = joint_name_to_index_.size();
    robot_state_.tree_.getJointNames(joint_name_to_index_,robot_state_.tree_.joint_name_to_index_);
    robot_state_.tree_.getTreeJointIndex(robot_state_.tree_.kdl_tree_, robot_state_.tree_.tree_joint_index_);

    q_current_.resize(num_joints_);
    q_current_.data.setZero();

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

    loadParameterFiles();

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

    // Initialize Joint Limit Avoidance
    //for (uint i = 0; i < num_joints_; i++) ROS_INFO("JLA gain of joint %i is %f",i,JLA_gain[i]);
    JointLimitAvoidance_.initialize(q_min, q_max, JLA_gain, JLA_workspace);
    ROS_INFO("Joint limit avoidance initialized");

    // Initialize Posture Controller
    PostureControl_.initialize(q_min, q_max, posture_q0, posture_gain, joint_name_to_index_);
    ROS_INFO("Posture Control initialized");

    /// Resizing variables (are set to zero in updatehook)
    qdot_reference_.resize(num_joints_);
    q_reference_.resize(num_joints_);
    tau_.resize(num_joints_);
    Jacobians_.resize(4);
    Ns_.resize(4);
    taus_.resize(4);
    for (unsigned int i = 0; i < 4; i++) {
        Jacobians_[i].resize(100, num_joints_);
        Ns_[i].resize(num_joints_, num_joints_);
        taus_[i].resize(num_joints_);
    }

    /// Initialize tracer
    std::vector<std::string> column_names = index_to_joint_name_;
    for (unsigned int i = 0; i < index_to_joint_name_.size(); i++) column_names.push_back("tau_cart"+index_to_joint_name_[i]);
    for (unsigned int i = 0; i < index_to_joint_name_.size(); i++) column_names.push_back("tau_null"+index_to_joint_name_[i]);
    for (unsigned int i = 0; i < index_to_joint_name_.size(); i++) column_names.push_back("tau_total"+index_to_joint_name_[i]);
    column_names.push_back("cartesian_impedance_cost");
    column_names.push_back("collision_avoidance_cost");
    column_names.push_back("joint_limit_cost");
    column_names.push_back("posture_cost");
    int buffersize;
    std::string foldername;
    n.param<int> ("/whole_body_controller/tracing_buffersize", buffersize, 0);
    n.param<std::string> ("/whole_body_controller/tracing_folder", foldername, "/tmp/");
    std::string filename = "joints";
    //std::string folderfilename = foldername + filename; // ToDo: make nice
    tracer_.Initialize(foldername, filename, column_names, buffersize);
    statsPublisher_.initialize();

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
    statsPublisher_.startTimer("WholeBodyController::update");

    /// Set some variables to zero
    tau_.setZero();
    for (unsigned int i = 0; i < 4; i++) {
        Jacobians_[i].setZero();
        Ns_[i].setZero();
        taus_[i].setZero();
    }

    /// Update the kinematic tree and FK
    robot_state_.tree_.rearrangeJntArrayToTree(q_current_);
    robot_state_.collectFKSolutions();
    robot_state_.updateCollisionBodyPoses();

    /// Update motion objectives
    unsigned int row_index = 0;
    std::vector<unsigned int> row_indexes(4,0);
    Eigen::MatrixXd jacobian_tree(100, num_joints_);//ToDo: make nice (or member)
    jacobian_tree.setZero();
    for(std::vector<MotionObjective*>::iterator it_motionobjective = motionobjectives_.begin(); it_motionobjective != motionobjectives_.end(); ++it_motionobjective)
    {
        MotionObjective* motionobjective = *it_motionobjective;
        //ROS_INFO("Motion Objective: %p", motionobjective);

        statsPublisher_.startTimer("WholeBodyController::motionobjective::" + motionobjective->type_);

        motionobjective->apply(robot_state_);

        unsigned int priority = motionobjective->getPriority();
        taus_[priority-1] += motionobjective->getTorques();
        Eigen::MatrixXd partial_jacobian = motionobjective->getJacobian();
        int number_rows = partial_jacobian.rows();
        if ( (row_indexes[priority-1]+number_rows) < Jacobians_[priority-1].rows()) {
            Jacobians_[priority-1].block(row_index, 0, number_rows, num_joints_) = partial_jacobian;
            row_indexes[priority-1] += number_rows;
        } else {
            ROS_WARN("Number of rows of the Jacobian is getting too large, omitting Jacobian!!!");
        }

        statsPublisher_.stopTimer("WholeBodyController::motionobjective::" + motionobjective->type_);
    }

    /// Update other motion objectives
    // Joint limit avoidance currently has priority 4, hence:
    JointLimitAvoidance_.update(q_current_, taus_[3]);

    // Posture control: similar
    PostureControl_.update(q_current_, taus_[3]);

    /// Project torques of lower priority into nullspace of higher order Jacobians
    //Eigen::VectorXd tautemp = taus_[0];
    //Eigen::VectorXd tautemp2;
    for (int i = 2; i >= 0; i--) {
        ComputeNullspace_.update(Jacobians_[i].block(0,0,row_indexes[i],num_joints_), Ns_[i]);
        taus_[i] += Ns_[i] * taus_[i+1];
        //if (i == 0) tautemp2 = taus_[0];
    }
    tau_ = taus_[0]; // ToDo: do we need this???
    //std::cout << "Row indexes: " << row_indexes[0] << ", " << row_indexes[1] << ", " << row_indexes[2] << ", " << row_indexes[3] << std::endl;
    //for (unsigned int i = 0; i < num_joints_; i++) std::cout << "Joint " << i << ", before: " << tautemp[i] << ", after: " << tautemp2[i] << std::endl;

    /// Update the admittance controller
    AdmitCont_.update(tau_, qdot_reference_, q_current_, q_reference_);
    //for (unsigned int i = 0; i < index_to_joint_name_.size(); i++) ROS_INFO("%s [cur, des, tau, qdot]  = %f, %f, %f, %f", index_to_joint_name_[i].c_str(), q_current_(i), q_reference_(i), tau_(i), qdot_reference_(i));

    q_reference = q_reference_;
    qdot_reference = qdot_reference_;

    /// Only trace useful data (there is an Cartesian Impedance which has not yet converged)...
    bool do_trace = false;
    double ci_cost = 0.0;
    double ca_cost = 0.0;
    for (unsigned int i = 0; i < motionobjectives_.size(); i++) {
        if (motionobjectives_[i]->type_ == "CartesianImpedance") {
            ci_cost += motionobjectives_[i]->getCost();
            if (motionobjectives_[i]->getStatus() == 2) {
                do_trace = true;
            }
        }
        else if (motionobjectives_[i]->type_ == "CollisionAvoidance") {
            ca_cost += motionobjectives_[i]->getCost();
        }
    }
    if (do_trace) {
        tracer_.newLine();
        tracer_.collectTracing(1, q_current_.data);
        /////tracer_.collectTracing(num_joints_+1, tau_cart);
        /////tracer_.collectTracing(2*num_joints_+1, tau_null);
        tracer_.collectTracing(3*num_joints_+1, tau_);
        tracer_.collectTracing(4*num_joints_+1, ci_cost);
        tracer_.collectTracing(4*num_joints_+2, ca_cost);
        tracer_.collectTracing(4*num_joints_+3, JointLimitAvoidance_.getCost());
        tracer_.collectTracing(4*num_joints_+4, PostureControl_.getCost());
    }

    statsPublisher_.stopTimer("WholeBodyController::update");

    statsPublisher_.publish();

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

void WholeBodyController::loadParameterFiles()
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
            robot_state_.robot_.groups.push_back(robot_state_group);
        }
        if (robot_state_.robot_.groups.size() == 0)
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

                robot_state_.exclusion_checks.checks.push_back(exclusion);
            }
        }

        if (robot_state_.exclusion_checks.checks.size() == 0)
        {
            ROS_WARN("No exclusions from self-collision avoindance checks");
        }
        else if (robot_state_.exclusion_checks.checks.size() > 0)
        {
            ROS_DEBUG("Exclusions from self-collision checks are: ");
            for (std::vector<RobotState::Exclusion>::iterator it = robot_state_.exclusion_checks.checks.begin(); it != robot_state_.exclusion_checks.checks.end(); ++it)
            {
                RobotState::Exclusion excl = *it;
                ROS_DEBUG("Name body A = %s", excl.name_body_A.c_str());
                ROS_DEBUG("Name body B = %s", excl.name_body_B.c_str());
            }
        }

    } catch(XmlRpc::XmlRpcException& ex)
    {
        std::cout << ex.getMessage() << std::endl;
    }

}

std::map<std::string, unsigned int> WholeBodyController::getJointNameToIndex()
{
    return joint_name_to_index_;
}
