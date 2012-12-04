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

    /**
     * @brief Fills this chain with the joint measurements contained in the whole body joint vector
     * @param all_joint_measurements The full vector containing the joint positions of all joints in whole body
     */
    void setMeasuredJointPositions(const KDL::JntArray& all_joint_measurements);

    /**
     * @brief Computes the chain jacobian, then fills the whole body jacobian with the information from
     * the computed chain jacobian
     * @param jacobian The whole body jacobian that needs to be filled
     */
    void fillJacobian(Eigen::MatrixXd& jacobian) const;

    /**
     * @brief Adds all wrenches applied to this chain to a vector containing all wrenches of all chains
     * @param all_wrenches Reference to a vector containing all wrenches applied to all chains in the body
     */
    void fillCartesianWrench(Eigen::VectorXd& all_wrenches);

    unsigned int getNumJoints() const;

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

    KDL::ChainJntToJacSolver* jnt_to_jac_solver_;

    KDL::Chain kdl_chain_;

protected:

    /**
     * @brief Current positions of the joints in this chain
     */
    KDL::JntArray joint_positions_;

    /**
     * @brief Names of the joints in this chain
     */
    std::vector<std::string> joint_names_;

    /**
     * @brief Names of the links in this chain
     */
    std::vector<std::string> link_names_;

    /**
     * @brief Maps joint indices of this chain to joint indices of the whole body joint vector
     */
    std::vector<unsigned int> joint_chain_index_to_full_index_;

    std::map<std::string, Eigen::VectorXd> cartesian_wrenches_;

};

#endif



