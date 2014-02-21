#include "amigo_whole_body_controller/motionobjectives/CartesianImpedance.h"

#include <tf/transform_datatypes.h>

CartesianImpedance::CartesianImpedance(const std::string& tip_frame, const double Ts) {

    type_      = "CartesianImpedance";
    tip_frame_ = tip_frame;
    priority_  = 3;
    cost_      = 0.0;
    Ts_        = Ts;

    ROS_INFO("Initializing Cartesian Impedance for %s", tip_frame_.c_str());

    ros::NodeHandle n("~");

    /// Set stiffness matrix to zero
    K_.resize(6,6);
    K_.setZero();
    D_.resize(6,6);
    D_.setZero();
    num_constrained_dofs_ = 0;// ToDo: make this less beun

    for (unsigned int i = 0; i < 3; i++) {
        box_tolerance_[i] = 0;
        orientation_tolerance_[i] = 0;
    }
    sphere_tolerance_ = 0;
    cylinder_tolerance_[0] = 0;
    cylinder_tolerance_[1] = 0;

    pose_error_.Zero();
    frame_tip_offset.Identity();
    ee_map_vel_.Zero();

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
    //std::string folderfilename = foldername + filename;

    int buffersize;
    n.param<int> ("/whole_body_controller/tracing_buffersize", buffersize, 0);
    tracer_.Initialize(foldername, filename, column_names, buffersize);

    //ROS_INFO("Initialized Cartesian Impedance");
}

void CartesianImpedance::setVelocity(RobotState &robotstate){

    /// Find the pose of the goal frame
    robotstate.collectFKSolutions();

    /// Get end effector pose (is in map frame)
    std::map<std::string, KDL::Frame>::iterator itrFK = robotstate.fk_poses_.find(tip_frame_);
    KDL::Frame frame_map_tip  = (*itrFK).second;

    /// Include tip offset
    frame_map_tip =  frame_map_tip * frame_tip_offset;

    /// Get the pose of the root frame (of the goal) in map
    std::map<std::string, KDL::Frame>::iterator itrRF = robotstate.fk_poses_.find(root_frame_);
    KDL::Frame frame_map_root = itrRF->second;

    /// Convert end-effector pose to root
    KDL::Frame frame_root_tip = frame_map_root.Inverse() * frame_map_tip;

    /// Compute end-effector velocity in root frame
    KDL::Twist ee_vel_root = KDL::diff(frame_root_tip_previous_, frame_root_tip, Ts_);

    ROS_INFO("Previous end-effector pose = %f %f %f", frame_root_tip_previous_.p.x(), frame_root_tip_previous_.p.y(), frame_root_tip_previous_.p.z());
    ROS_INFO("Current  end-effector pose = %f %f %f", frame_root_tip.p.x(), frame_root_tip.p.y(), frame_root_tip.p.z());
    ROS_INFO("Refreshing the end-effector velocity = %f %f %f %f %f %f with sample time %f", ee_vel_root.vel.x(),ee_vel_root.vel.y(), ee_vel_root.vel.z(), ee_vel_root.rot.x(),ee_vel_root.rot.y(),ee_vel_root.rot.z(),Ts_);

    ref_generator_[0].setCurrentVelocity(ee_vel_root.vel.x());
    ref_generator_[1].setCurrentVelocity(ee_vel_root.vel.y());
    ref_generator_[2].setCurrentVelocity(ee_vel_root.vel.z());
    ref_generator_[3].setCurrentVelocity(ee_vel_root.rot.x());
    ref_generator_[4].setCurrentVelocity(ee_vel_root.rot.y());
    ref_generator_[5].setCurrentVelocity(ee_vel_root.rot.z());

}


bool CartesianImpedance::initialize(RobotState &robotstate) {

    /// Find the pose of the goal frame
    robotstate.collectFKSolutions();

    /// Get end effector pose (is in map frame)
    std::map<std::string, KDL::Frame>::iterator itrFK = robotstate.fk_poses_.find(tip_frame_);
    KDL::Frame frame_map_tip  = (*itrFK).second;

    /// Include tip offset
    frame_map_tip =  frame_map_tip * frame_tip_offset;
    frame_map_ee_previous_ = frame_map_tip;

    /// Get the pose of the root frame (of the goal) in map
    std::map<std::string, KDL::Frame>::iterator itrRF = robotstate.fk_poses_.find(root_frame_);
    KDL::Frame frame_map_root = itrRF->second;

    /// Convert end-effector pose to root
    KDL::Frame frame_root_tip = frame_map_root.Inverse() * frame_map_tip;

    /// Init the reference generators
    double roll, pitch, yaw;
    frame_root_tip.M.GetRPY(roll, pitch, yaw);

    ref_generator_.resize(6);
    ref_generator_[0] = controller::RefGenerator(frame_root_tip.p.x(), 0.5, 0.8);
    ref_generator_[1] = controller::RefGenerator(frame_root_tip.p.y(), 0.5, 0.8);
    ref_generator_[2] = controller::RefGenerator(frame_root_tip.p.z(), 0.5, 0.8);
    ref_generator_[3] = controller::RefGenerator(roll, 0.5, 0.8);
    ref_generator_[4] = controller::RefGenerator(pitch, 0.5, 0.8);
    ref_generator_[5] = controller::RefGenerator(yaw, 0.5, 0.8);

    /// Initialize vector and matrix objects
    unsigned int number_joints = robotstate.getNrJoints();
    jacobian_.resize(0,number_joints);
    jacobian_.setZero();
    jacobian_pre_alloc_.resize(6,number_joints);
    jacobian_pre_alloc_.setZero();
    wrenches_pre_alloc_.resize(6);
    wrenches_pre_alloc_.setZero();
    torques_.resize(number_joints);
    torques_.setZero();

    return true;
}

CartesianImpedance::~CartesianImpedance() {
}

void CartesianImpedance::setGoal(const geometry_msgs::PoseStamped& goal_pose ) {

    /// Root frame
    root_frame_ = goal_pose.header.frame_id;
    // ToDo: get rid of this!
    if (root_frame_ == "/amigo/base_link") root_frame_ = "base_link";

    stampedPoseToKDLframe(goal_pose, frame_root_goal_);

    ROS_DEBUG("Cartesian Impedance: set goal %s w.r.t. %s", tip_frame_.c_str(), root_frame_.c_str() );
    ROS_WARN("Cartesian Impedance: set goal at %f, %f, %f w.r.t. %s",frame_root_goal_.p.x(), frame_root_goal_.p.y(), frame_root_goal_.p.z(), root_frame_.c_str());
}

void CartesianImpedance::setGoalOffset(const geometry_msgs::Point& target_point_offset ) {

    ROS_DEBUG("Target point offset (x,y,z): %f, %f, %f", target_point_offset.x, target_point_offset.y, target_point_offset.z);
    ROS_WARN_ONCE("Why set offset twice???");
    frame_tip_offset.p.x(target_point_offset.x);
    frame_tip_offset.p.y(target_point_offset.y);
    frame_tip_offset.p.z(target_point_offset.z);

}

void CartesianImpedance::setImpedance(const geometry_msgs::Wrench &stiffness) {

    /// Copy desired stiffness into K matrix
    K_(0,0) = stiffness.force.x;
    K_(1,1) = stiffness.force.y;
    K_(2,2) = stiffness.force.z;
    K_(3,3) = stiffness.torque.x;
    K_(4,4) = stiffness.torque.y;
    K_(5,5) = stiffness.torque.z;

    /// Damping matrix
    // Controller zero at s = -k/b.
    // Desired to be at 1/3 of the bandwidth
    // Assuming bw = 1.0 --> b = 3k
    // ToDo: refine?
    //for (unsigned int i = 0; i < 6; i++) D_(i,i) = 0.01 * K_(i,i);
    for (unsigned int i = 0; i < 6; i++) D_(i,i) = 0.1 * K_(i,i);

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

    /// Reset stuff
    jacobian_pre_alloc_.setZero();
    wrenches_pre_alloc_.setZero();
    torques_.setZero();

    /// Get end effector pose (this is in map frame)
    std::map<std::string, KDL::Frame>::iterator itrFK = robotstate.fk_poses_.find(tip_frame_);
    KDL::Frame frame_map_tip  = (*itrFK).second;
    //std::cout << "Frame tip in map: x = " << frame_map_tip.p.x() << ", y = " << frame_map_tip.p.y() << ", z = " << frame_map_tip.p.z() << std::endl;

    /// Include tip offset
    KDL::Frame frame_map_ee =  frame_map_tip * frame_tip_offset;
    //std::cout << "Frame tip in map incl offset: x = " << frame_map_ee.p.x() << ", y = " << frame_map_ee.p.y() << ", z = " << frame_map_ee.p.z() << std::endl;

    /// Get the pose of the root frame (of the goal) in map
    std::map<std::string, KDL::Frame>::iterator itrRF = robotstate.fk_poses_.find(root_frame_);
    KDL::Frame frame_map_root = itrRF->second;
    //double r, p, y;frame_map_root.M.GetRPY(r,p,y);
    //std::cout << "Frame root in map: x = " << frame_map_root.p.x() << ", y = " << frame_map_root.p.y() << ", z = " << frame_map_root.p.z() <<
    //             ", roll = " << r << ", pitch = " << p << ", yaw = " << y << std::endl;

    /// Generate the setpoint for the Cartesian impedance
    KDL::Frame frame_root_ref;
    refGeneration(frame_root_goal_, frame_root_ref);
    //std::cout << "Frame goal in root: x = " << frame_root_ref.p.x() << ", y = " << frame_root_ref.p.y() << ", z = " << frame_root_ref.p.z() << std::endl;

    /// Convert end-effector pose to root
    KDL::Frame frame_root_ee = frame_map_root.Inverse() * frame_map_ee;
    //std::cout << "Frame ee in root: x = " << frame_root_ee.p.x() << ", y = " << frame_root_ee.p.y() << ", z = " << frame_root_ee.p.z() << std::endl;

    /// Convert goal pose in root to goal pose in map
    //KDL::Frame frame_map_goal = frame_map_root * frame_root_ref;
    //std::cout << "Frame interpolated goal in map: x = " << frame_map_goal.p.x() << ", y = " << frame_map_goal.p.y() << ", z = " << frame_map_goal.p.z() << std::endl;

    /// Compute pose error in root coordinates w.r.t. end_goal). This is NOT required for the computation!
    KDL::Frame frame_root_tip = frame_map_root.Inverse() * frame_map_ee;
    pose_error_ = KDL::diff(frame_root_tip , frame_root_goal_);

    /// Compute pose error in root coordinates w.r.t. current (=interpolated) setpoint
    KDL::Twist ref_root_error = KDL::diff(frame_root_ee, frame_root_ref);
    //std::cout << "Error in root frame: x = " << ref_root_error.vel.x() << ", y = " << ref_root_error.vel.y() << ", z = " << ref_root_error.vel.z() << std::endl;

    /// Convert error to map frame
    KDL::Twist ref_map_error = frame_map_root.M * ref_root_error;
    //std::cout << "Error in map frame: x = " << ref_map_error.vel.x() << ", y = " << ref_map_error.vel.y() << ", z = " << ref_map_error.vel.z() << std::endl;

    /// Compute end-effector velocity in map frame
    KDL::Twist ee_vel_map = KDL::diff(frame_map_ee_previous_, frame_map_ee, Ts_);
    //std::cout << "End-effector velocity in map frame is x: " << ee_vel_map.vel.x() << ", y: " << ee_vel_map.vel.y() << ", z: " << ee_vel_map.vel.z() << std::endl;
    frame_map_ee_previous_ = frame_map_ee;

    /// Compute the corresponding forces
    // ToDo: is this correct??? K_ is typically represented in a different frame!
    // Should K_ be in map, root, end-effector frame or should we switch back to a 1 DoF force representation?
    Eigen::VectorXd F_task(6);
    F_task(0) = K_(0,0) * ref_map_error.vel.x() - D_(0,0) * ee_vel_map.vel.x();
    F_task(1) = K_(1,1) * ref_map_error.vel.y() - D_(1,1) * ee_vel_map.vel.y();
    F_task(2) = K_(2,2) * ref_map_error.vel.z() - D_(2,2) * ee_vel_map.vel.z();
    F_task(3) = K_(3,3) * ref_map_error.rot.x() - D_(3,3) * ee_vel_map.rot.x();
    F_task(4) = K_(4,4) * ref_map_error.rot.y() - D_(4,4) * ee_vel_map.rot.x();
    F_task(5) = K_(5,5) * ref_map_error.rot.z() - D_(5,5) * ee_vel_map.rot.x();
    //std::cout << "F in map frame: x = " << F_task(0) << ", y = " << F_task(1) << ", z = " << F_task(2) << std::endl;

    /// Compute 6xn Jacobian (this is w.r.t.root frame of the kinematic tree, so in this case: base_link)
    Eigen::MatrixXd temp_jacobian(6,robotstate.getNrJoints());// ToDo: get rid of this
    KDL::Jacobian partial_jacobian(robotstate.getNrJoints());
    robotstate.tree_.calcPartialJacobian(tip_frame_, temp_jacobian);
    //ROS_INFO("Tip frame = %s", tip_frame_.c_str());
    partial_jacobian.data = temp_jacobian;
    //for (unsigned int i = 0; i < robotstate.getNrJoints(); i++) std::cout << "Jacobian (1," << i+1 << ") = " << temp_jacobian(0,i) << std::endl;

    /// Change reference point: the force does not always act at the end of a certain link
    KDL::Twist tip_offset_map = KDL::diff(frame_map_tip, frame_map_ee);
    partial_jacobian.changeRefPoint(KDL::Vector(tip_offset_map.vel.x(), tip_offset_map.vel.y(), tip_offset_map.vel.z()));
    //for (unsigned int i = 0; i < robotstate.getNrJoints(); i++) std::cout << "Jacobian (1," << i+1 << ") = " << partial_jacobian.data(0,i) << std::endl;

    /// Change base: the Jacobian is computed w.r.t. base_link instead of map, while the force is expressed in map
    std::map<std::string, KDL::Frame>::iterator itrBLF = robotstate.fk_poses_.find("base_link"); //ToDo: don't hardcode???
    KDL::Frame BaseFrame_in_map = itrBLF->second;
    partial_jacobian.changeBase(BaseFrame_in_map.M);
    //for (unsigned int i = 0; i < robotstate.getNrJoints(); i++) std::cout << "Jacobian (3," << i+1 << ") = " << partial_jacobian.data(2,i) << std::endl;

    /// Copy the desired rows in one matrix
    unsigned int row_index = 0;

    // ToDo: doublecheck!
    /// Choose either of the following three options

    /// 1: Translations and rotations
    for (unsigned int i = 0; i < 6; i++) {
        if (K_(i,i) > 0.0) {
            jacobian_pre_alloc_.block(row_index, 0, 1, robotstate.getNrJoints()) = partial_jacobian.data.block(i, 0, 1, robotstate.getNrJoints());
            wrenches_pre_alloc_(row_index) = F_task(i);
            ++row_index;
        }
    }
/*
    /// 2: In case of translations: only one DoF needs to be constrained
    if (K_(0,0) > 0.0 || K_(1,1) > 0.0 || K_(2,2) > 0.0) {
        double amplitude = sqrt( F_task(0)*F_task(0) + F_task(1)*F_task(1) + F_task(2)*F_task(2) );
        Eigen::Vector3d force_direction(F_task(0)/amplitude, F_task(1)/amplitude, F_task(2)/amplitude);
        jacobian_pre_alloc_.block(row_index, 0, 1, robotstate.getNrJoints()) = force_direction.transpose() * partial_jacobian.data.block(0, 0, 3, robotstate.getNrJoints());
        wrenches_pre_alloc_(row_index) = amplitude;
        //std::cout << "Jacobian force row = " << jacobian_pre_alloc_.block(row_index, 0, 1, robotstate.getNrJoints()) << std::endl;
        ++row_index;
    }
    /// Rotations
    if (K_(3,3) > 0.0 || K_(4,4) > 0.0 || K_(5,5) > 0.0) {
        double amplitude = sqrt( F_task(3)*F_task(3) + F_task(4)*F_task(4) + F_task(5)*F_task(5) );
        Eigen::Vector3d force_direction(F_task(3)/amplitude, F_task(4)/amplitude, F_task(5)/amplitude);
        jacobian_pre_alloc_.block(row_index, 0, 1, robotstate.getNrJoints()) = force_direction.transpose() * partial_jacobian.data.block(3, 0, 3, robotstate.getNrJoints());
        wrenches_pre_alloc_(row_index) = amplitude;
        //std::cout << "Jacobian force row = " << jacobian_pre_alloc_.block(row_index, 0, 1, robotstate.getNrJoints()) << std::endl;
        ++row_index;
    }
*/
/*
    /// 3: Combine all DoFs
    double amplitude = sqrt( F_task(0)*F_task(0) + F_task(1)*F_task(1) + F_task(2)*F_task(2) + F_task(3)*F_task(3) + F_task(4)*F_task(4) + F_task(5)*F_task(5) );
    Eigen::VectorXd force_direction(6);
    for (unsigned int i = 0;i < 6; i++) force_direction(i) = F_task(i)/amplitude;
    jacobian_pre_alloc_.block(row_index, 0, 1, robotstate.getNrJoints()) = force_direction.transpose() * partial_jacobian.data.block(0, 0, 6, robotstate.getNrJoints());
    wrenches_pre_alloc_(row_index) = amplitude;
    ++row_index;
*/
    /// Extract total Jacobian and vector with wrenches
    jacobian_ = jacobian_pre_alloc_.block(0, 0, row_index, robotstate.getNrJoints());
    Eigen::VectorXd wrenches = wrenches_pre_alloc_.block(0, 0, row_index, 1);
    //std::cout << "Wrenches: \n" << wrenches_new_ << std::endl;

    /// Multiply to get torques
    torques_ = jacobian_.transpose() * wrenches;

    /// Compute costs
    cost_ = F_task.norm();
    //std::cout << "Torque due to Cartesian impedance = " << torques_ << std::endl;

    /// Log data when active
    if (status_ == 2) {
        tracer_.newLine();
        tracer_.collectTracing(1,  frame_root_ref);
        //tracer_.collectTracing(7,  frame_root_tip);
        tracer_.collectTracing(7,  frame_root_tip * frame_tip_offset.Inverse());
        tracer_.collectTracing(13, ref_root_error);
        tracer_.collectTracing(19, F_task);
    }

    if (convergedConstraints() >= num_constrained_dofs_ && status_ == 2) {
        status_ = 1;
        tracer_.writeToFile();
        //ROS_WARN("Converged!, remaining error (x,y,z,r,p,y) = (%f, %f, %f, %f, %f, %f)",error_vector(0),error_vector(1),error_vector(2),error_vector(3),error_vector(4),error_vector(5));
    }

	/// Set the tip velocity
    frame_root_tip_previous_ = frame_root_tip;

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
    error_vector(3) = pose_error_.rot.x();
    error_vector(4) = pose_error_.rot.y();
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

    double roll, pitch, yaw;
    goal.M.GetRPY(roll, pitch, yaw);

    /// Generate
    ref_generator_[0].generate(goal.p.x(), 0.02, false);
    ref_generator_[1].generate(goal.p.y(), 0.02, false);
    ref_generator_[2].generate(goal.p.z(), 0.02, false);
    ref_generator_[3].generate(roll,  0.02, false);
    ref_generator_[4].generate(pitch, 0.02, false);
    ref_generator_[5].generate(yaw,   0.02, false);

    /// Assign
    ref.p.x(ref_generator_[0].getPositionReference());
    ref.p.y(ref_generator_[1].getPositionReference());
    ref.p.z(ref_generator_[2].getPositionReference());
    ref.M = KDL::Rotation::RPY(ref_generator_[3].getPositionReference(), ref_generator_[4].getPositionReference(), ref_generator_[5].getPositionReference());

}
