#include "amigo_whole_body_controller/motionobjectives/CartesianImpedance.h"
#include <tf/transform_datatypes.h>

CartesianImpedance::CartesianImpedance(const std::string& tip_frame) :
    chain_(0) {

    type_ = "CartesianImpedance";
    tip_frame_ = tip_frame;

    ROS_INFO("Initializing Cartesian Impedance for %s", tip_frame_.c_str());

    ros::NodeHandle n("~");

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
    ref_tip_offset.Zero();

    /// Initialize tracer
    std::vector<std::string> column_names;
    column_names.push_back("rx");
    column_names.push_back("ry");
    column_names.push_back("rz");
    column_names.push_back("rroll");
    column_names.push_back("rpitch");
    column_names.push_back("ryaw");
    column_names.push_back("x");
    column_names.push_back("y");
    column_names.push_back("z");
    column_names.push_back("roll");
    column_names.push_back("pitch");
    column_names.push_back("yaw");
    column_names.push_back("ex");
    column_names.push_back("ey");
    column_names.push_back("ez");
    column_names.push_back("eroll");
    column_names.push_back("epitch");
    column_names.push_back("eyaw");
    column_names.push_back("Fx");
    column_names.push_back("Fy");
    column_names.push_back("Fz");
    column_names.push_back("Froll");
    column_names.push_back("Fpitch");
    column_names.push_back("Fyaw");

    std::string filename("CartImp");
    std::string foldername;
    n.param<std::string> ("/whole_body_controller/tracing_foldername", foldername, "/tmp/");
    std::string folderfilename = foldername + filename;

    int buffersize;
    n.param<int> ("/whole_body_controller/tracing_buffersize", buffersize, 0);
    tracer_.Initialize(folderfilename, column_names, buffersize);

    ROS_INFO("Initialized Cartesian Impedance");
}

bool CartesianImpedance::initialize(RobotState &robotstate) {

    /// Find the pose of the goal frame
    robotstate.collectFKSolutions();

    /// Get end effector pose (is in map frame)
    std::map<std::string, KDL::Frame>::iterator itrFK = robotstate.fk_poses_.find(tip_frame_);
    KDL::Frame frame_map_tip  = (*itrFK).second;

    /// Include tip offset
    frame_map_tip =  frame_map_tip * Frame_tip_offset;

    /// Get the pose of the root frame (of the goal) in map
    std::map<std::string, KDL::Frame>::iterator itrRF = robotstate.fk_poses_.find(root_frame_);
    KDL::Frame frame_map_root = itrRF->second;

    /// Convert end-effector pose to root
    KDL::Frame frame_root_tip = frame_map_root.Inverse() * frame_map_tip;

    /// Init the reference generators
    ref_generators.resize(6);
    ref_generators[0].setRefGen(frame_root_tip.p.x());
    ref_generators[1].setRefGen(frame_root_tip.p.y());
    ref_generators[2].setRefGen(frame_root_tip.p.z());

    double roll, pitch, yaw;
    frame_root_tip.M.GetRPY(roll, pitch, yaw);
    ref_generators[3].setRefGen(roll);
    ref_generators[4].setRefGen(pitch);
    ref_generators[5].setRefGen(yaw);

    ROS_DEBUG("Initializing refgens at [%f,\t%f,\t%f,\t%f,\t%f,\t%f,\t]", frame_root_tip.p.x(), frame_root_tip.p.y(), frame_root_tip.p.z(), roll, pitch, yaw);

    return true;
}

CartesianImpedance::~CartesianImpedance() {
}

void CartesianImpedance::setGoal(const geometry_msgs::PoseStamped& goal_pose ) {

    /// Root frame
    root_frame_ = goal_pose.header.frame_id;

    stampedPoseToKDLframe(goal_pose, Frame_root_goal_);

    /// Maximum magnitude of the repulsive force
    // ToDo: get rid of hardcoding
    f_max_ = 75;

    ROS_INFO("Cartesian Impedance: set goal %s w.r.t. %s", tip_frame_.c_str(), root_frame_.c_str() );
}

void CartesianImpedance::setGoalOffset(const geometry_msgs::Point& target_point_offset ) {

    ROS_DEBUG("Target point offset (x,y,z): %f, %f, %f", target_point_offset.x, target_point_offset.y, target_point_offset.z);

    Frame_tip_offset.p.x(target_point_offset.x);
    Frame_tip_offset.p.y(target_point_offset.y);
    Frame_tip_offset.p.z(target_point_offset.z);

    ref_tip_offset.x(target_point_offset.x);
    ref_tip_offset.y(target_point_offset.y);
    ref_tip_offset.z(target_point_offset.z);

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
    tracer_.writeToFile();
    status_ = 0;
}

void CartesianImpedance::apply(RobotState &robotstate) {

    /// Get end effector pose (is in map frame)
    std::map<std::string, KDL::Frame>::iterator itrFK = robotstate.fk_poses_.find(tip_frame_);
    KDL::Frame frame_map_tip  = (*itrFK).second;
    //ROS_INFO("FK pose tip frame in map  = (%f,%f,%f)", Frame_map_tip_.p.x(), Frame_map_tip_.p.y(), Frame_map_tip_.p.z());

    /// Include tip offset
    frame_map_tip =  frame_map_tip * Frame_tip_offset;

    /// Get the pose of the root frame (of the goal) in map
    std::map<std::string, KDL::Frame>::iterator itrRF = robotstate.fk_poses_.find(root_frame_);
    KDL::Frame Frame_map_root = itrRF->second;
    //ROS_INFO("FK pose root frame in map (x,y,z) = (%f,%f,%f)", Frame_map_root.p.x(), Frame_map_root.p.y(), Frame_map_root.p.z());

    /// Convert end-effector pose to root
    KDL::Frame frame_root_tip = Frame_map_root.Inverse() * frame_map_tip;
    //ROS_INFO("Tip frame expressed in root [x, y, z] = [%f,\t%f,\t%f]",frame_root_tip.p.x(),frame_root_tip.p.y(),frame_root_tip.p.z());

    /// Generate a reference for the cartesian impedance
    KDL::Frame frame_root_ref;
    refGeneration(Frame_root_goal_, frame_root_ref);

    /// Compute pose error
    pose_error_ = KDL::diff(frame_root_tip , Frame_root_goal_);
    KDL::Twist ref_pose_error = KDL::diff(frame_root_tip , frame_root_ref);
    //ROS_INFO("Error [x, y, z, roll, pitch, yaw] = [%f,\t%f,\t%f,\t%f,\t%f,\t%f", pose_error_.vel.x(), pose_error_.vel.y(), pose_error_.vel.z(), pose_error_.rot.x(), pose_error_.rot.y(), pose_error_.rot.z());

    /// Error between tip and reference frame
    //ROS_WARN("Error [x,y,z] = %f\t%f\t%f", pose_error_.vel.x(), pose_error_.vel.y(), pose_error_.vel.z());
    Eigen::VectorXd error_vector(6);
    error_vector(0) = ref_pose_error.vel.x();
    error_vector(1) = ref_pose_error.vel.y();
    error_vector(2) = ref_pose_error.vel.z();
    error_vector(3) = ref_pose_error.rot.x();
    error_vector(4) = ref_pose_error.rot.y();
    error_vector(5) = ref_pose_error.rot.z();

    /// Compute Force in map frame
    Eigen::VectorXd F_task = K_ * error_vector;

    /// Transform to KDL::Wrench
    KDL::Wrench W_task;
    W_task.force.x(F_task(0));
    W_task.force.y(F_task(1));
    W_task.force.z(F_task(2));
    W_task.torque.x(F_task(3));
    W_task.torque.y(F_task(4));
    W_task.torque.z(F_task(5));

    /// Change the reference point from offset, to tip. No need to change Frame.
    W_task = W_task.RefPoint(-ref_tip_offset);

    /// Transform to map
    std::map<std::string, double> W_task_map;
    if (K_(0,0) > 0.0) W_task_map["x"] = W_task.force.x();
    if (K_(1,1) > 0.0) W_task_map["y"] = W_task.force.y();
    if (K_(2,2) > 0.0) W_task_map["z"] = W_task.force.z();
    if (K_(3,3) > 0.0) W_task_map["rx"] = W_task.torque.x();
    if (K_(4,4) > 0.0) W_task_map["ry"] = W_task.torque.y();
    if (K_(5,5) > 0.0) W_task_map["rz"] = W_task.torque.z();
    //ROS_INFO("Wtask = %f, %f, %f, %f, %f, %f", W_task.force.x(), W_task.force.y(), W_task.force.z(), W_task.torque.x(), W_task.torque.y(), W_task.torque.z());

    /// Log data
    tracer_.newLine();
    tracer_.collectTracing(1,  frame_root_ref);
    tracer_.collectTracing(7,  frame_root_tip);
    tracer_.collectTracing(13, ref_pose_error);
    tracer_.collectTracing(19, W_task);

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

    /// Add the wrench to the end effector of the kinematic chain
    robotstate.tree_.addCartesianWrench(tip_frame_, W_task_map);

    if (convergedConstraints() == num_constrained_dofs_ && status_ == 2) {
        status_ = 1;
        tracer_.writeToFile();
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
        double radius2 = error_vector(0)*error_vector(0) + error_vector(1)*error_vector(1);
        if (radius2 < (cylinder_tolerance_[0]*cylinder_tolerance_[0]) ) {
            num_converged_dof += 2;
        }
        if (fabs(error_vector(2)) < cylinder_tolerance_[1]) {
            num_converged_dof += 1;
        }
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
