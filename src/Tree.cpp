#include "Tree.h"

using namespace std;

Tree::Tree() {
}

Tree::~Tree() {
}

void Tree::fillJacobian(Eigen::MatrixXd& jacobian)
{
    //std::cout << cartesian_wrenches_.size() << std::endl;
    /// If no Cartesian wrenches: return
    if (cartesian_wrenches_.empty()) {
        return;
    }

    /// Iterate over all wrenches to count required number of DoFs
    int number_constrained_dofs = 0;
    for(std::map<std::string, std::map<std::string, double> >::iterator it_wrench = cartesian_wrenches_.begin(); it_wrench != cartesian_wrenches_.end(); ++it_wrench) {
        /// Iterate over constrained DoFs
        std::map<std::string, double> wrench = it_wrench->second;
        for (std::map<std::string, double>::iterator it_dof = wrench.begin(); it_dof != wrench.end(); ++it_dof) {
            ++number_constrained_dofs;
        }
    }

    /// Resize Jacobian
    jacobian.resize(number_constrained_dofs, number_of_joints_);
    jacobian.setZero();

    /// Loop over wrenches, compute partial Jacobian and fill in!
    unsigned int row_index = 0;
    for(std::map<std::string, std::map<std::string, double> >::iterator it_wrench = cartesian_wrenches_.begin(); it_wrench != cartesian_wrenches_.end(); ++it_wrench)
    {
        std::pair<std::string, std::map<std::string, double> > wrench = *it_wrench;

        if (tree_joint_index_.size() == 0){
            ROS_WARN("Tree joint index is empty");
        }

        /// Compute partial Jacobian (6xn)
        Eigen::MatrixXd partial_jacobian(6, number_of_joints_);
        partial_jacobian.setZero();
        calcPartialJacobian(wrench.first, partial_jacobian);

        /// Loop over degrees of freedom. If constrained: fill in!
        std::map<std::string, double>::iterator dof_iter;
        std::string dofs[6] = {"x", "y", "z", "roll", "pitch", "yaw"};
        for (unsigned j = 0; j < 6; j++) {
            dof_iter = wrench.second.find(dofs[j]);
            if (dof_iter != wrench.second.end()) {
                for (int i = 0; i < number_of_joints_; i++) {
                    jacobian(row_index, i) = partial_jacobian(0, i);
                }
                ++row_index;
            }
        }

        //unsigned int link_start_index = jacobian.rows();
        //Eigen::MatrixXd jacobian_new(jacobian.rows() + 6, jacobian.cols());
        //jacobian_new.setZero();

        // Add 6 new rows to the current whole body jacobian (TODO: could be much more efficient)
        /*
        for(unsigned int i = 0; i < jacobian.rows(); ++i) {
            for(unsigned int j = 0; j < jacobian.cols(); ++j) {
                jacobian_new(i, j) = jacobian(i, j);
            }
        }
        jacobian = jacobian_new;

        for(unsigned int i = 0; i < partial_jacobian.rows(); ++i)
        {
            for(unsigned int j = 0; j < partial_jacobian.cols(); ++j)
            {
                jacobian(link_start_index + i, j) = partial_jacobian(i,j);
            }
        }*/
    }
}

void Tree::fillCartesianWrench(Eigen::VectorXd& all_wrenches) {

    /// Indicates where to start adding wrenches
    unsigned int link_start_index = all_wrenches.rows();

    /// Iterate over all wrenches to count required number of DoFs
    int number_constrained_dofs = 0;
    for(std::map<std::string, std::map<std::string, double> >::iterator it_wrench = cartesian_wrenches_.begin(); it_wrench != cartesian_wrenches_.end(); ++it_wrench) {
        /// Iterate over constrained DoFs
        std::map<std::string, double> wrench = it_wrench->second;
        for (std::map<std::string, double>::iterator it_dof = wrench.begin(); it_dof != wrench.end(); ++it_dof) {
            ++number_constrained_dofs;
        }
    }

    Eigen::VectorXd wrench_new(all_wrenches.rows() + number_constrained_dofs);
    wrench_new.setZero();

    // Add 6 new values to current whole body wrench vector (TODO: could be much more efficient)

    /// Copy all currently present forces/torques
    for(unsigned int i = 0; i < all_wrenches.rows(); ++i) {
        wrench_new(i) = all_wrenches(i);
    }
    all_wrenches = wrench_new;

    /// Fill all_wrenches with the wrenches of this link
    for(std::map<std::string, std::map<std::string, double> >::iterator it_wrench = cartesian_wrenches_.begin(); it_wrench != cartesian_wrenches_.end(); ++it_wrench)
    {
        std::map<std::string, double> wrench = it_wrench->second;
        for (std::map<std::string, double>::iterator it_dof = wrench.begin(); it_dof != wrench.end(); ++it_dof) {
            all_wrenches(link_start_index) = it_dof->second;
            ++link_start_index;
        }
    }

    //cout << "torque = " << endl;
    //cout << torque << endl;
}

void Tree::addCartesianWrench(const std::string& link_name, const Eigen::VectorXd& wrench)
{
    std::map<std::string, std::map<std::string, double> >::iterator it_wrench = cartesian_wrenches_.find(link_name);

    /// Check size of input vector
    std::map<std::string, double> wrench_map;
    if (wrench.rows() != 6) {
        ROS_WARN("addCartesianWrench: please use correct format");
        return;
    } else {
        wrench_map["x"] = wrench(0);
        wrench_map["y"] = wrench(1);
        wrench_map["z"] = wrench(2);
        wrench_map["roll"] = wrench(3);
        wrench_map["pitch"] = wrench(4);
        wrench_map["yaw"] = wrench(5);
    }

    if (it_wrench == cartesian_wrenches_.end()) {
        /// If wrench does not exist yet: make a new one
        cartesian_wrenches_[link_name] = wrench_map;
    } else {
        /// Check if DoFs are already constrained
        for (std::map<std::string, double>::iterator wrench_iter = wrench_map.begin(); wrench_iter != wrench_map.end(); ++wrench_iter) {
            std::map<std::string, double>::iterator index = cartesian_wrenches_[link_name].find(wrench_iter->first);
            if (index == cartesian_wrenches_[link_name].end()) {
                cartesian_wrenches_[link_name][wrench_iter->first] = wrench_iter->second;
            } else {
                cartesian_wrenches_[link_name][wrench_iter->first] += wrench_iter->second;
            }
        }
    }
    //std::cout << cartesian_wrenches_.size() << std::endl;
}

void Tree::removeCartesianWrenches() {
    cartesian_wrenches_.clear();
}

void Tree::calcPartialJacobian(std::string& link_name,
                               Eigen::MatrixXd& jacobian)
{
    KDL::Jacobian kdl_jacobian;
    kdl_jacobian.resize(kdl_tree_.getNrOfJoints());

    //std::cout << q_tree.data.size() << std::endl;
    //std::cout << tree_joint_index_.size() << std::endl;

    jac_solver_->JntToJac(q_tree_,kdl_jacobian,link_name);

    //std::cout << kdl_jacobian.rows() << std::endl;
    //std::cout << kdl_jacobian.columns() << std::endl;
    /*
    std::cout << "tree_joint_index_" << std::endl;
    for (std::vector<int>::iterator it = tree_joint_index_.begin(); it != tree_joint_index_.end(); ++it)
    {
        std::cout << *it << std::endl;
    }
    */

    for(unsigned int i = 0; i < kdl_jacobian.rows(); ++i)
    {
        for(unsigned int j = 0; j < kdl_jacobian.columns(); ++j)
        {
            if (tree_joint_index_[j] >= 0)
            {
                jacobian(i,tree_joint_index_[j]) = kdl_jacobian(i,j);
            }
        }
    }

    /*
    Eigen::MatrixXd printJ(kdl_jacobian.rows(),kdl_jacobian.columns());
    for(unsigned int i = 0; i < kdl_jacobian.rows(); ++i)
    {
        for(unsigned int j = 0; j < kdl_jacobian.columns(); ++j)
        {
            printJ(i, j) = kdl_jacobian(i, j);
        }
    }

    //std::map<std::string,KDL::TreeElement>::const_iterator it_segment = kdl_tree_.getRootSegment();
    //std::pair<std::string,KDL::TreeElement> pair = *it_segment;

    //cout << "root segment = " << pair.first << std::endl;
    //cout << "link_name = " << link_name << std::endl;
    //cout << "q_in = " << endl << q_tree.data << endl;
    //cout << "printJ = " << endl << printJ << endl;
    //cout << "partial_jacobian = " << endl << jacobian << endl;
    */
}

void Tree::getJointNames(std::map<string, unsigned int> &jnt_name_to_index_in,std::map<string, unsigned int> &jnt_name_to_index_out)
{
    jnt_name_to_index_out = jnt_name_to_index_in;
    joint_name_to_index_ = jnt_name_to_index_in;
    number_of_joints_ = jnt_name_to_index_in.size();
}

void Tree::getTreeJointIndex(KDL::Tree& tree, std::vector<int>& tree_joint_index)
{
    std::map<std::string,KDL::TreeElement> segments_map = tree.getSegments();

    tree_joint_index.clear();
    tree_joint_index.resize(tree.getNrOfJoints());

    for(std::map<std::string,KDL::TreeElement>::const_iterator it_segment = segments_map.begin(); it_segment != segments_map.end(); ++it_segment)
    {
        std::pair<std::string,KDL::TreeElement> pair = *it_segment;
        std::map<std::string,KDL::TreeElement>::const_iterator segment_itr = tree.getSegment(pair.first);
        std::pair<std::string,KDL::TreeElement> segment = *segment_itr;

        uint q_nr = segment.second.q_nr;
        const KDL::Joint joint = segment.second.segment.getJoint();

        if (joint.getType() != KDL::Joint::None)
        {
            std::map<std::string, unsigned int>::iterator it_names = joint_name_to_index_.find(joint.getName());
            if (it_names == joint_name_to_index_.end())
            {
                tree_joint_index[q_nr] = -1;
                //std::cout << joint.getName() << ": " << -1 << std::endl;
            }
            else
            {
                std::pair<std::string, unsigned int> name = *it_names;
                tree_joint_index[q_nr] = name.second;
                //std::cout << joint.getName() << ": " << name.second << std::endl;
            }
        }
    }
}

void Tree::rearrangeJntArrayToTree(KDL::JntArray& q_in)
{
    q_tree_.resize(kdl_tree_.getNrOfJoints());
    uint i = 0;

    for(std::vector<int>::iterator it_index = tree_joint_index_.begin(); it_index != tree_joint_index_.end(); ++it_index, ++i)
    {
        int index = *it_index;
        if (index < 0 )
        {
            q_tree_.data[i] = 0; // Unused joint values are set to zero
        }
        else if (index >= 0 )
        {
            q_tree_.data[i] = q_in.data[index];
        }
    }
}
