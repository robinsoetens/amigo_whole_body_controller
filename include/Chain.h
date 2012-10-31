#ifndef WBC_CHAIN_H_
#define WBC_CHAIN_H_

#include <vector>

#include <kdl/chainjnttojacsolver.hpp>

class Component;

class Chain {

public:

    Chain();

    virtual ~Chain();

    std::vector<Component*> components_;

    KDL::Chain kdl_chain_;

    KDL::ChainJntToJacSolver* jnt_to_jac_solver_;

    void addComponent(const std::string& component_name);

};

#endif
