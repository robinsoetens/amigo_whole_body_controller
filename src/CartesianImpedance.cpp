#include "CartesianImpedance.h"
#include <tf/transform_datatypes.h>

#include <amigo_whole_body_controller/ArmTaskAction.h>
#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Float64MultiArray.h>


CartesianImpedance::CartesianImpedance(const std::string& tip_frame) :
    chain_(0) {

    type_ = "CartesianImpedance";
    tip_frame_ = tip_frame;

    ROS_INFO("Initializing Cartesian Impedance for %s", tip_frame_.c_str());

    ros::NodeHandle n("~");
    pub_CI_wrench_ = n.advertise<std_msgs::Float64MultiArray>("/whole_body_controller/ci_wrench/", 10);

    /// Set stiffness matrix to zero
    K_.resize(6,6);
    K_.setZero();
    num_constrained_dofs_ = 0;

    for (unsigned int i = 0; i < 3; i++) {
        box_tolerance_[i] = 0;
        orientation_tolerance_[i] = 0;
    }
    sphere_tolerance_ = 0;
    cylinder_tolerance_[0] = cylinder_tolerance_[1] = 0;

    pose_error_.Zero();
    ref_pose_error_.Zero();
    ref_tip_offset.Zero();

    ROS_INFO("Initialized Cartesian Impedance");
}

bool CartesianImpedance::initialize(RobotState &robotstate) {

    /// Find the pose of the goal frame
    robotstate.collectFKSolutions();
    std::map<std::string, KDL::Frame>::iterator itrFK = robotstate.fk_poses_.find(tip_frame_);
    Frame_map_tip_ = (*itrFK).second;

    if(pre_grasp){
        Frame_map_tip_ =  Frame_map_tip_* Frame_tip_offset;
    }

    /// Init the reference generators
    ref_generators.resize(6);
    ref_generators[0].setRefGen(Frame_map_tip_.p.x());
    ref_generators[1].setRefGen(Frame_map_tip_.p.y());
    ref_generators[2].setRefGen(Frame_map_tip_.p.z());


    double roll, pitch, yaw;
    Frame_map_tip_.M.GetRPY(roll, pitch, yaw);
    ref_generators[3].setRefGen(roll);
    ref_generators[4].setRefGen(pitch);
    ref_generators[5].setRefGen(yaw);
    return true;
}

CartesianImpedance::~CartesianImpedance() {
}

void CartesianImpedance::setGoal(const geometry_msgs::PoseStamped& goal_pose ) {

    /// Root frame
    root_frame_ = goal_pose.header.frame_id;

    stampedPoseToKDLframe(goal_pose, Frame_root_goal_);
    pre_grasp = false;

    /// Maximum magnitude of the repulsive force
    // ToDo: get rid of hardcoding
    f_max_ = 75;
}

void CartesianImpedance::setGoalOffset(const geometry_msgs::Point& target_point_offset ) {

    ROS_INFO("PRE-GRAPSP: Received target point offset (x,y,z): %f, %f, %f", target_point_offset.x, target_point_offset.y, target_point_offset.z);

    Frame_tip_offset.p.x(-target_point_offset.x);
    Frame_tip_offset.p.y(-target_point_offset.y);
    Frame_tip_offset.p.z(-target_point_offset.z);

    ref_tip_offset.x(target_point_offset.x);
    ref_tip_offset.y(target_point_offset.y);
    ref_tip_offset.z(target_point_offset.z);

    pre_grasp = true;
}


void CartesianImpedance::setImpedance(const geometry_msgs::Wrench &stiffness) {

    /// Copy desired stiffness into K matrix
    K_(0,0) = stiffness.force.x;
    K_(1,1) = stiffness.force.y;
    K_(2,2) = stiffness.force.z;
    K_(3,3) = stiffness.torque.x;
    K_(4,4) = stiffness.torque.y;
    K_(5,5) = stiffness.torque.z;

    /// Count the number of constrained DoFs
    num_constrained_dofs_ = 0;
    for (unsigned int i = 0; i < 6; i++) {
        if (K_(i,i) != 0) {
            num_constrained_dofs_ += 1;
        }
    }
   /// Set the status of current impedance
   status_ = 2;
}

bool CartesianImpedance::setPositionTolerance(const arm_navigation_msgs::Shape &position_tolerance) {

    constraint_type_ = position_tolerance.type;
    if (position_tolerance.type == 0) {
        if (!position_tolerance.dimensions.empty()) sphere_tolerance_ = position_tolerance.dimensions[0];
        else {
            ROS_INFO("Impedance: Defaulting to position constraint: sphere with radius 0.03 [m].");
            sphere_tolerance_ = 0.03;
        }
        return true;
    }
    else if (position_tolerance.type == 1) {
        box_tolerance_[0] = position_tolerance.dimensions[0];
        box_tolerance_[1] = position_tolerance.dimensions[1];
        box_tolerance_[2] = position_tolerance.dimensions[2];
        return true;
    }
    else if (position_tolerance.type == 2){
        cylinder_tolerance_[0] = position_tolerance.dimensions[0];
        cylinder_tolerance_[1] = position_tolerance.dimensions[1];
        return true;
    }
    else {
        ROS_WARN("Other position tolerance than sphere/box/cylinder not yet implemented, defaulting");
        return false;
    }
}

bool CartesianImpedance::setOrientationTolerance(const float roll, const float pitch, const float yaw) {
    //ToDo default values in function parameters
    if (roll == 0) orientation_tolerance_[0] = 0.3;
    else orientation_tolerance_[0] = roll;
    if (pitch == 0) orientation_tolerance_[1] = 0.3;
    else orientation_tolerance_[1] = pitch;
    if (yaw == 0) orientation_tolerance_[2] = 0.3;
    else orientation_tolerance_[2] = yaw;
    return true;
}

void CartesianImpedance::cancelGoal() {
    status_ = 0;
}

void CartesianImpedance::apply(RobotState &robotstate) {

    Eigen::VectorXd F_task_root(6);
    Eigen::VectorXd F_task_map(6);
    KDL::Wrench F_task_wrench_root;
    KDL::Wrench F_task_wrench_map;
    Eigen::VectorXd error_vector(6);

    // ToDo: create get function
    /// Get end effector pose (is in map frame)
    std::map<std::string, KDL::Frame>::iterator itrFK = robotstate.fk_poses_.find(tip_frame_);
    Frame_map_tip_  = (*itrFK).second;
    //ROS_INFO("FK pose tip frame in map  = (%f,%f,%f)", Frame_map_tip_.p.x(), Frame_map_tip_.p.y(), Frame_map_tip_.p.z());

    if(pre_grasp){

        ROS_INFO_ONCE("Tip at (x,y,z): %f, %f, %f", Frame_map_tip_.p.x(), Frame_map_tip_.p.y(), Frame_map_tip_.p.z());
        Frame_map_tip_ =  Frame_map_tip_* Frame_tip_offset;
        ROS_INFO_ONCE("Offset at (x,y,z): %f, %f, %f", Frame_map_tip_.p.x(), Frame_map_tip_.p.y(), Frame_map_tip_.p.z());

    }

    /// Get the pose of the root frame (of the goal) in map
    std::map<std::string, KDL::Frame>::iterator itrRF = robotstate.fk_poses_.find(root_frame_);
    KDL::Frame Frame_map_root = itrRF->second;
    //ROS_INFO("FK pose root frame in map (x,y,z) = (%f,%f,%f)", Frame_map_root.p.x(), Frame_map_root.p.y(), Frame_map_root.p.z());

    /// Convert the goal pose to map (is required because pose of robot in map (amcl_pose) may have been updated)
    KDL::Frame Frame_map_goal = Frame_map_root * Frame_root_goal_;
    //ROS_INFO("FK pose of goal in map (x,y,z) = (%f,%f,%f)",Frame_map_goal.p.x(),Frame_map_goal.p.y(),Frame_map_goal.p.z());

    /// Generate a reference for the cartesian impedance
    KDL::Frame Frame_map_ref;
    refGeneration(Frame_map_goal, Frame_map_ref);
    //ROS_INFO("FK pose of ref in map (x,y,z) = (%f,%f,%f)",Frame_map_ref.p.x(),Frame_map_ref.p.y(),Frame_map_ref.p.z());

    /// Compute pose error
    pose_error_ = KDL::diff(Frame_map_tip_ , Frame_map_goal);
    ref_pose_error_ = KDL::diff(Frame_map_tip_ , Frame_map_ref);

    /// Error between tip and reference frame
    error_vector(0) = ref_pose_error_.vel.x();
    error_vector(1) = ref_pose_error_.vel.y();
    error_vector(2) = ref_pose_error_.vel.z();
    error_vector(3) = ref_pose_error_.rot.x();//goal_RPY(0) - end_effector_RPY(0)/ Ts;
    error_vector(4) = ref_pose_error_.rot.y();//goal_RPY(1) - end_effector_RPY(1)/ Ts;
    error_vector(5) = ref_pose_error_.rot.z();//goal_RPY(2) - end_effector_RPY(2)/ Ts;

    //std::cout << "error_vector = " << error_vector << std::endl;

    /// Compute Force in map frame
    F_task_map = K_ * error_vector;

    /// Transform to KDL::Wrench
    F_task_wrench_map.force.x(F_task_map(0));
    F_task_wrench_map.force.y(F_task_map(1));
    F_task_wrench_map.force.z(F_task_map(2));
    F_task_wrench_map.torque.x(F_task_map(3));
    F_task_wrench_map.torque.y(F_task_map(4));
    F_task_wrench_map.torque.z(F_task_map(5));

    //std::cout<<"Before Trans: "<<F_task_map(0)<<" "<<F_task_map(1)<<" "<<F_task_map(2)<<" "<<F_task_map(3)<<" "<<F_task_map(4)<<" "<<F_task_map(5)<<"\n";

    /// Transform Force from map to root. Maybe efficienter to check if amcl change is big enough to afford to recalculate new Inverse.
    F_task_wrench_root = Frame_map_root.Inverse() * F_task_wrench_map;

    /// Now check if we have a pre_grasp goal

    if(pre_grasp)
    {
        std::cout<<"Offset forces: "<<F_task_wrench_root.force.x()<<" "<<F_task_wrench_root.force.y()<<" "<<F_task_wrench_root.force.z()<<" "<<F_task_wrench_root.torque.x()<<" "<<F_task_wrench_root.torque.y()<<" "<<F_task_wrench_root.torque.z()<<std::endl;
        /// Change the reference point from offset, to tip. No need to change Frame.
        F_task_wrench_root = F_task_wrench_root.RefPoint(-ref_tip_offset);
    }


    /// Transform to Eigen::VectorXd
    F_task_root(0) = F_task_wrench_root.force.x();
    F_task_root(1) = F_task_wrench_root.force.y();
    F_task_root(2) = F_task_wrench_root.force.z();
    F_task_root(3) = F_task_wrench_root.torque.x();
    F_task_root(4) = F_task_wrench_root.torque.y();
    F_task_root(5) = F_task_wrench_root.torque.z();

    //std::cout<<"EE forces in root: "<<F_task_root(0)<<" "<<F_task_root(1)<<" "<<F_task_root(2)<<" "<<F_task_root(3)<<" "<<F_task_root(4)<<" "<<F_task_root(5)<<"\n\n";

    /*
    /// Limit the maximum magnitude of the attractive force so it will always be smaller than the maximum magnitude of the repulisive force generated by the collision avoidance
    for (uint i = 0; i < 3; i++)
    {
        if (F_task_root(i) > f_max_ )
        {
            F_task_root(i) = f_max_;
        }
        else if (F_task_root(i) < -f_max_ )
        {
            F_task_root(i) = -f_max_;
        }
    }
    */
    //std::cout << "K_ = " << K_ << std::endl;
    //std::cout << "F_task = " << F_task << std::endl;
    //ROS_INFO("Tip frame = %s",tip_frame_.c_str());

    // add the wrench to the end effector of the kinematic chain
    robotstate.tree_.addCartesianWrench(tip_frame_,F_task_root);

    if (tip_frame_ == "grippoint_left")
    {
        // Publish the wrench
        std_msgs::Float64MultiArray msgW;
        for (uint i = 0; i < F_task_root.rows(); i++)
        {
            msgW.data.push_back(F_task_root(i));
        }
        pub_CI_wrench_.publish(msgW);
    }

    if (convergedConstraints() == num_constrained_dofs_ && status_ == 2) {
        status_ = 1;
        ROS_WARN("Converged!, remaining error (x,y,z,r,p,y) = (%f, %f, %f, %f, %f, %f)",error_vector(0),error_vector(1),error_vector(2),error_vector(3),error_vector(4),error_vector(5));
    }
}

unsigned int CartesianImpedance::getStatus() {
    return status_;
}

KDL::Twist CartesianImpedance::getError() {
    return pose_error_;
}

void CartesianImpedance::stampedPoseToKDLframe(const geometry_msgs::PoseStamped& pose, KDL::Frame& frame) {

    //if (pose.header.frame_id != "base_link") //ROS_WARN("FK computation can now only cope with base_link as input frame, while it currently is %s", pose.header.frame_id.c_str());

    // Position
    frame.p.x(pose.pose.position.x);
    frame.p.y(pose.pose.position.y);
    frame.p.z(pose.pose.position.z);

    // Orientation
    frame.M = KDL::Rotation::Quaternion(pose.pose.orientation.x,
                                        pose.pose.orientation.y,
                                        pose.pose.orientation.z,
                                        pose.pose.orientation.w);
}

unsigned int CartesianImpedance::convergedConstraints(){
    unsigned int num_converged_dof = 0;
    Eigen::VectorXd error_vector(6);
    error_vector(0) = pose_error_.vel.x();
    error_vector(1) = pose_error_.vel.y();
    error_vector(2) = pose_error_.vel.z();
    error_vector(3) = pose_error_.rot.x();//goal_RPY(0) - end_effector_RPY(0)/ Ts;
    error_vector(4) = pose_error_.rot.y();//goal_RPY(1) - end_effector_RPY(1)/ Ts;
    error_vector(5) = pose_error_.rot.z();

    /// Orientation
    for (unsigned int i = 3; i < 6; i++) {
        if (K_(i,i) != 0) {
            if (fabs(error_vector(i)) < orientation_tolerance_[i-3]) ++num_converged_dof;
        }
    }

    /// Position
    if (constraint_type_ == 0) //Sphere
    {
        if ( (error_vector(0)*error_vector(0) + error_vector(1)*error_vector(1) + error_vector(2)*error_vector(2)) < sphere_tolerance_*sphere_tolerance_ ) num_converged_dof = num_converged_dof + 3;
    }

    else if (constraint_type_ == 1) //Box
    {
        for (unsigned int i = 0; i < 3; i++) {
            if (K_(i,i) != 0) {
                if (fabs(error_vector(i)) < box_tolerance_[i]/2) ++num_converged_dof;
            }
        }
    }

    else if (constraint_type_ == 2) //Cylinder, what is the height axis?, how to define this in the message?
    {
        ROS_INFO("CYLINDER NOT IMPLEMENTED");
    }
    else if (constraint_type_ == 3) //Mesh
    {
        ROS_INFO("MESH NOT IMPLEMENTED");
    }
    return num_converged_dof;
}

void CartesianImpedance::refGeneration(KDL::Frame& goal, KDL::Frame& ref)
{
    //ToDo assign appropriate velocities and accelerations, now v_max 0.5 and a_max = 0.8
    std::vector<amigo_msgs::ref_point> ref_points (6);
    double roll, pitch, yaw;

    ref_points[0] = ref_generators[0].generateReference(goal.p.x(), 0.5, 0.8, 0.02, false, 0);
    ref.p.x(ref_points[0].pos);

    ref_points[1] = ref_generators[1].generateReference(goal.p.y(), 0.5, 0.8, 0.02, false, 0);
    ref.p.y(ref_points[1].pos);

    ref_points[2] = ref_generators[2].generateReference(goal.p.z(), 0.5, 0.8, 0.02, false, 0);
    ref.p.z(ref_points[2].pos);

    goal.M.GetRPY(roll, pitch, yaw);
    ref_points[3] = ref_generators[3].generateReference(roll, 0.5, 0.8, 0.02, false, 0);
    ref_points[4] = ref_generators[4].generateReference(pitch, 0.5, 0.8, 0.02, false, 0);
    ref_points[5] = ref_generators[5].generateReference(yaw, 0.5, 0.8, 0.02, false, 0);

    ref.M = KDL::Rotation::RPY(ref_points[3].pos, ref_points[4].pos, ref_points[5].pos);
}
