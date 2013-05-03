/*!
 * \author Paul Metsemakers
 * \date January, 2013
 * \version 0.1
 */

#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <map>
#include "Chain.h"
#include "Tree.h"
#include <XmlRpc.h>

// Messages
#include <geometry_msgs/PoseStamped.h>

// KDL
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

// Bullet
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>


class RobotState
{

public:

    //Constructor
    RobotState();

    //Destructor
    virtual ~RobotState();

    std::map<std::string, geometry_msgs::PoseStamped> fk_poses_;

    // Collision Model
    struct CollisionBody
    {
        std::string name_collision_body;
        struct CollisionShape
        {
            std::string shape_type;
            struct Dimensions
            {
                double x;
                double y;
                double z;
            } dimensions;
        } collision_shape;
        std::string chain_side;
        geometry_msgs::PoseStamped fk_pose;
        geometry_msgs::PoseStamped fix_pose;
        btTransform bt_transform;
        btConvexShape* bt_shape;

        void fromXmlRpc(XmlRpc::XmlRpcValue& value)
        {
            name_collision_body = static_cast<std::string>(value["name"]);

            // COLLISION SHAPE
            XmlRpc::XmlRpcValue collShape = value["shape"];
            collision_shape.shape_type =  static_cast<std::string>(collShape["shape_type"]);

            XmlRpc::XmlRpcValue dim = collShape["dimensions"];
            collision_shape.dimensions.x = dim["x"];
            collision_shape.dimensions.y = dim["y"];
            collision_shape.dimensions.z = dim["z"];

            // CORRECTION TRANSFORM
            XmlRpc::XmlRpcValue corrTransform = value["transform"];
            XmlRpc::XmlRpcValue origin = corrTransform["origin"];
            XmlRpc::XmlRpcValue orientation = corrTransform["orientation"];

            chain_side = static_cast<std::string>(corrTransform["chain_side"]);

            fix_pose.header.frame_id = static_cast<std::string>(corrTransform["frame_id"]);
            fix_pose.pose.position.x = origin["x"];
            fix_pose.pose.position.y = origin["y"];
            fix_pose.pose.position.z = origin["z"];
            fix_pose.pose.orientation.x = orientation["x"];
            fix_pose.pose.orientation.y = orientation["y"];
            fix_pose.pose.orientation.z = orientation["z"];
            fix_pose.pose.orientation.w = orientation["w"];
        }
    } collision_body;

    struct Exclusion
    {
        std::string frame_id_A;
        std::string frame_id_B;

        void fromXmlRpc(XmlRpc::XmlRpcValue& value)
        {
            frame_id_A = static_cast<std::string>(value["FrameIdBodyA"]);
            frame_id_B = static_cast<std::string>(value["FrameIdBodyB"]);
        }

    } exclusion;

    struct Robot
    {
        std::vector< std::vector<CollisionBody> > groups;
    } robot_;

    struct ExclusionChecks
    {
        std::vector<Exclusion> checks;
    } exclusion_checks;


    geometry_msgs::PoseStamped poseGrippointLeft_;
    geometry_msgs::PoseStamped poseGrippointRight_;

    /**
     * Jacobian Solver
     */
    KDL::Tree tree_;
    KDL::TreeJntToJacSolver* jac_solver_;
    std::vector<KDL::TreeJntToJacSolver*> jac_solvers_;

    Chain* chain_left_;
    Chain* chain_right_;
    std::vector<Chain*> chains_;

    /**
      * Forward Kinematics Solver
      */
    std::vector<KDL::ChainFkSolverPos_recursive*> fk_solvers;
    KDL::ChainFkSolverPos_recursive* fk_solver_left;
    KDL::ChainFkSolverPos_recursive* fk_solver_right;

    /**
      * Store all FK solutions in a map
      */
    void collectFKSolutions(std::vector<Chain*> &chains, std::map<std::string, geometry_msgs::PoseStamped> &fk_poses);

    /**
      * computeForwardKinematics
      */
    void computeForwardKinematics(KDL::Frame &FK_end_effector_pose, const std::string &chain_side, int segmentNr, RobotState &robot_state);

    /**
     * Create the forward kinematics solvers for both chains
     */
    void separateChains(const std::vector<Chain *> &chains, RobotState &robot_state);

    /**
      * Returns the current FK solution
      *
      */
    void getFKsolution(KDL::Frame& FK_pose, geometry_msgs::PoseStamped &pose);

    /**
      * Function returns the positions of the end-effectors
      */
    void getFK(std::vector<geometry_msgs::PoseStamped>& poses);


    int getNrOfSegment(KDL::Chain kdl_chain_, const std::string& segment_name);

    void calcPartialJacobian(std::string& frame_id, KDL::TreeJntToJacSolver *jac_solver_, Eigen::MatrixXd& jacobian);

};

#endif // ROBOTSTATE_H


