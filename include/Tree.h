#ifndef TREE_H
#define TREE_H

#include <vector>
#include <map>

#include <kdl/treejnttojacsolver.hpp>

class Tree {

public:

    Tree();

    virtual ~Tree();

    KDL::Tree kdl_tree_;
    KDL::TreeJntToJacSolver* tree_jnt_to_jac_solver_;

//protected:

};

#endif // TREE_H
