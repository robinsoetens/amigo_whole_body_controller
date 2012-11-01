#ifndef WBC_CHAIN_H_
#define WBC_CHAIN_H_

#include <vector>

#include <kdl/chainjnttojacsolver.hpp>

class Component;

class Chain {

public:

    Chain();

    virtual ~Chain();

    void addComponent(Component* component);

    const std::vector<Component*>& getComponents() const;

    KDL::Jacobian getJacobian() const;

    Component* getRootComponent() const;

    Component* getLeafComponent() const;

    unsigned int getNumJoints() const;

    void addEndEffectorTorque(const Eigen::VectorXd& torque);

    const Eigen::VectorXd& getEndEffectorTorque();

    void removeEndEffectorTorque();

protected:

    std::vector<Component*> components_;

    unsigned int num_joints_;

    std::string root_link_;

    std::string leaf_link_;

    KDL::Chain kdl_chain_;

    KDL::ChainJntToJacSolver* jnt_to_jac_solver_;

    Eigen::VectorXd end_effector_torque_;



};

#endif



