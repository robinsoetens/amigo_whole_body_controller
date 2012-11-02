#ifndef WBC_CHAIN_H_
#define WBC_CHAIN_H_

#include <vector>
#include <map>

#include <kdl/chainjnttojacsolver.hpp>

class Component;

class Chain {

public:

    Chain();

    virtual ~Chain();

    void addJoint(const std::string& joint_name, const std::string& link_name, unsigned int full_joint_index);

    bool hasLink(const std::string& link_name) const;

    void setMeasuredJointPositions(const KDL::JntArray& all_joint_measurements);

    void addToJacobian(Eigen::MatrixXd& jacobian) const;

    void addToCartesianTorque(Eigen::VectorXd& torque);

    unsigned int getNumJoints() const;

    void addCartesianTorque(const std::string& link_name, const Eigen::VectorXd& torque);

    void removeCartesianTorques();

    KDL::ChainJntToJacSolver* jnt_to_jac_solver_;

    KDL::Chain kdl_chain_;

protected:

    KDL::JntArray joint_positions_;

    std::vector<std::string> joint_names_;

    std::vector<std::string> link_names_;

    std::vector<unsigned int> joint_chain_index_to_full_index_;

    std::map<std::string, Eigen::VectorXd> cartesian_torques_;

    /*
    std::vector<Component*> components_;

    unsigned int num_joints_;

    std::string root_link_;

    std::string leaf_link_;

    Eigen::VectorXd end_effector_torque_;

    */

};

#endif



