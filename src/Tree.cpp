#include "Tree.h"

using namespace std;

Tree::Tree() {
}

Tree::~Tree() {
}

void Tree::fillJacobian(Eigen::MatrixXd& jacobian)
{
    //std::cout << cartesian_wrenches_.size() << std::endl;
    if (cartesian_wrenches_.empty()) {
        return;
    }

    for(std::map<std::string, std::map<std::string, double> >::iterator it_wrench = cartesian_wrenches_.begin(); it_wrench != cartesian_wrenches_.end(); ++it_wrench)
    {
        // ToDo: entire function way more efficient
        std::pair<std::string, std::map<std::string, double> > wrench = *it_wrench;

        if (tree_joint_index_.size() == 0){
            ROS_WARN("Tree joint index is empty");
        }

        /// Computes 6xn Jacobian
        Eigen::MatrixXd partial_jacobian(6, number_of_joints_);
        partial_jacobian.setZero();
        calcPartialJacobian(wrench.first,partial_jacobian);

        /// Create new Jacobian of the correct size
        unsigned int link_start_index = jacobian.rows();
        Eigen::MatrixXd jacobian_new(jacobian.rows() + wrench.second.size(), jacobian.cols());
        jacobian_new.setZero();

        /// Copy data of old to new Jacobian
        jacobian_new.block(0,0,jacobian.rows(),jacobian.cols()) = jacobian;

        /// Make old new again
        jacobian = jacobian_new;

        /// Copy data from partial Jacobian: only copy data if DoF is present
        unsigned int add_index = 0;
        if (wrench.second.find("x") != wrench.second.end()) {
            jacobian.block(link_start_index + add_index, 0, 1, number_of_joints_) = partial_jacobian.block(0, 0, 1, number_of_joints_);
            ++add_index;
        }
        if (wrench.second.find("y") != wrench.second.end()) {
            jacobian.block(link_start_index + add_index, 0, 1, number_of_joints_) = partial_jacobian.block(1, 0, 1, number_of_joints_);
            ++add_index;
        }
        if (wrench.second.find("z") != wrench.second.end()) {
            jacobian.block(link_start_index + add_index, 0, 1, number_of_joints_) = partial_jacobian.block(2, 0, 1, number_of_joints_);
            ++add_index;
        }
        if (wrench.second.find("rx") != wrench.second.end()) {
            jacobian.block(link_start_index + add_index, 0, 1, number_of_joints_) = partial_jacobian.block(3, 0, 1, number_of_joints_);
            ++add_index;
        }
        if (wrench.second.find("ry") != wrench.second.end()) {
            jacobian.block(link_start_index + add_index, 0, 1, number_of_joints_) = partial_jacobian.block(4, 0, 1, number_of_joints_);
            ++add_index;
        }
        if (wrench.second.find("rz") != wrench.second.end()) {
            jacobian.block(link_start_index + add_index, 0, 1, number_of_joints_) = partial_jacobian.block(5, 0, 1, number_of_joints_);
            ++add_index;
        }
    }
}

void Tree::fillCartesianWrench(Eigen::VectorXd& all_wrenches) {

    /// Loop over wrenches to determine size
    unsigned int num_constrained_dofs = 0;
    for (std::map<std::string, std::map<std::string, double> >::iterator it_wrench = cartesian_wrenches_.begin(); it_wrench != cartesian_wrenches_.end(); ++it_wrench) {
        num_constrained_dofs += it_wrench->second.size();
    }

    /// Resize vector
    all_wrenches.resize(num_constrained_dofs);

    /// Loop over wrenches again to fill in the numbers
    unsigned int index = 0;
    for (std::map<std::string, std::map<std::string, double> >::iterator it_wrench = cartesian_wrenches_.begin(); it_wrench != cartesian_wrenches_.end(); ++it_wrench) {

        if (it_wrench->second.find("x") != it_wrench->second.end()) {
            all_wrenches(index) = it_wrench->second["x"];
            ++index;
        }
        if (it_wrench->second.find("y") != it_wrench->second.end()) {
            all_wrenches(index) = it_wrench->second["y"];
            ++index;
        }
        if (it_wrench->second.find("z") != it_wrench->second.end()) {
            all_wrenches(index) = it_wrench->second["z"];
            ++index;
        }
        if (it_wrench->second.find("rx") != it_wrench->second.end()) {
            all_wrenches(index) = it_wrench->second["rx"];
            ++index;
        }
        if (it_wrench->second.find("ry") != it_wrench->second.end()) {
            all_wrenches(index) = it_wrench->second["ry"];
            ++index;
        }
        if (it_wrench->second.find("rz") != it_wrench->second.end()) {
            all_wrenches(index) = it_wrench->second["rz"];
            ++index;
        }
    }
}

void Tree::addCartesianWrench(const std::string& link_name, const std::map<std::string, double>& wrench) {

    /// See if there's already a wrench for this link
    std::map<std::string, std::map<std::string, double> >::iterator it_wrench = cartesian_wrenches_.find(link_name);
    //ROS_INFO("Link %s", link_name.c_str());
    /// If not: create a new one
    if (it_wrench == cartesian_wrenches_.end()) {
        cartesian_wrenches_[link_name] = wrench;
        //for (std::map<std::string, double>::const_iterator iter = wrench.begin(); iter != wrench.end(); ++iter) ROS_INFO("DoF: %s = %f", iter->first.c_str(), iter->second);
    } else {
        /// Iterate over DoFs of input wrench
        for (std::map<std::string, double>::const_iterator iter_new = wrench.begin(); iter_new !=wrench.end(); ++iter_new) {

            /// Check whether this DoF is already constrained (iter_new->first = ConstrainedDoF, iter_new->second = value)
            std::map< std::string, double>::iterator iter_old = it_wrench->second.find(iter_new->first);

            /// If not: add new constrained DoF
            if (iter_old == it_wrench->second.end()) {
                it_wrench->second[iter_new->first] = iter_new->second;
            } else {
                it_wrench->second[iter_new->first] += iter_new->second;
            }
        }
    }
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
	// ToDo: why is this so difficult?
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
