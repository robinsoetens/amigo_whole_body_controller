#ifndef WBC_CHAIN_H_
#define WBC_CHAIN_H_

#include <vector>
#include <map>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/treejnttojacsolver.hpp>

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

    unsigned int getNumJoints() const;

    KDL::ChainJntToJacSolver* chain_jnt_to_jac_solver_;
    KDL::Chain kdl_chain_;

    /**
     * @brief Current positions of the joints in this chain
     */
    KDL::JntArray joint_positions_;

    /**
     * @brief Names of the joints in this chain
     */
    std::vector<std::string> joint_names_;

protected:

    /**
     * @brief Names of the links in this chain
     */
    std::vector<std::string> link_names_;

    /**
     * @brief Maps joint indices of this chain to joint indices of the whole body joint vector
     */
    std::vector<unsigned int> joint_chain_index_to_full_index_;

};

#endif



