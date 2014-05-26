#ifndef TREE_H
#define TREE_H

#include <vector>
#include <map>

#include <kdl/treejnttojacsolver.hpp>

class Tree {

public:

    Tree();

    virtual ~Tree();

    void calcPartialJacobian(const std::string &link_name, Eigen::MatrixXd &jacobian);

    void getJointNames(std::map<std::string, unsigned int>& jnt_name_to_index_in, std::map<std::string, unsigned int> &jnt_name_to_index_out);

    void getTreeJointIndex(KDL::Tree& tree, std::vector<int>& tree_joint_index);

    void rearrangeJntArrayToTree(KDL::JntArray &q_in);

    unsigned int getNrJoints();

    KDL::Tree kdl_tree_;

    /** Jacobian Solver */
    KDL::TreeJntToJacSolver* jac_solver_;

    std::map<std::string, unsigned int> joint_name_to_index_;

    std::vector<int> tree_joint_index_;

    KDL::JntArray q_tree_;

protected:

    /** Number of joints in this tree */
    int number_of_joints_;

};

#endif // TREE_H
