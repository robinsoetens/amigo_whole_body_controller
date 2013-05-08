#ifndef TREE_H
#define TREE_H

#include "Component.h"

#include <vector>
#include <map>

#include <kdl/treejnttojacsolver.hpp>

class Tree {

public:

    Tree();

    virtual ~Tree();

    /**
     * @brief Computes the chain jacobian, then fills the whole body jacobian with the information from
     * the computed chain jacobian
     * @param jacobian The whole body jacobian that needs to be filled
     */
    void fillJacobian(KDL::JntArray& joint_positions_, Eigen::MatrixXd& jacobian);

    /**
     * @brief Adds all wrenches applied to this chain to a vector containing all wrenches of all chains
     * @param all_wrenches Reference to a vector containing all wrenches applied to all chains in the body
     */
    void fillCartesianWrench(Eigen::VectorXd& all_wrenches);

    /**
     * @brief Adds a wrench in cartesian space (force + torque) to a link in the chain
     * @param link_name The name of the link to which the wrench should be applied
     * @param wrench The wrench to apply
     */
    void addCartesianWrench(const std::string& link_name, const Eigen::VectorXd& wrench);

    /**
     * @brief Remove all wrenches from all links in this chain
     */
    void removeCartesianWrenches();

    void calcPartialJacobian(std::string& link_name, KDL::JntArray& q_in, Eigen::MatrixXd jacobian);

    void getJointNames(std::map<std::string, unsigned int>& jnt_name_to_index_in, std::map<std::string, unsigned int> &jnt_name_to_index_out);

    void getTreeJointIndex(KDL::Tree& tree, std::vector<int>& tree_joint_index);

    KDL::Tree kdl_tree_;

    /**
     * Jacobian Solver
     */
    KDL::TreeJntToJacSolver* jac_solver_;
    std::vector<KDL::TreeJntToJacSolver*> jac_solvers_;

    KDL::TreeJntToJacSolver* tree_jnt_to_jac_solver_;

    std::map<std::string, unsigned int> joint_name_to_index_;

    std::vector<int> tree_joint_index_;

    /**
     * @brief Names of the joints in this chain
     */
    std::vector<std::string> joint_names_;

protected:

    /**
     * @brief Names of the links in the tree
     */
    std::vector<std::string> link_names_;

    std::map<std::string, Eigen::VectorXd> cartesian_wrenches_;

    int number_of_joints_;

};

#endif // TREE_H
