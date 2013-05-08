#include "Tree.h"


using namespace std;

Tree::Tree() {
}

Tree::~Tree() {
}

void Tree::fillJacobian(KDL::JntArray &joint_positions_, Eigen::MatrixXd& jacobian)
{
    //std::cout << cartesian_wrenches_.size() << std::endl;
    if (cartesian_wrenches_.empty()) {
        return;
    }

    for(std::map<std::string, Eigen::VectorXd>::iterator it_wrench = cartesian_wrenches_.begin(); it_wrench != cartesian_wrenches_.end(); ++it_wrench)
    {
        std::pair<std::string, Eigen::VectorXd> wrench = *it_wrench;

        if (tree_joint_index_.size() == 0){
            ROS_WARN("Tree joint index is empty");
        }

        Eigen::MatrixXd partial_jacobian(6, number_of_joints_);
        partial_jacobian.setZero();
        calcPartialJacobian(wrench.first,joint_positions_,partial_jacobian);

        Eigen::MatrixXd jacobian_new(jacobian.rows() + 6, jacobian.cols());
        jacobian_new.setZero();
        unsigned int link_start_index = jacobian.rows();

        // Add 6 new rows to the current whole body jacobian (TODO: could be much more efficient)
        for(unsigned int i = 0; i < jacobian.rows(); ++i) {
            for(unsigned int j = 0; j < jacobian.cols(); ++j) {
                jacobian_new(i, j) = jacobian(i, j);
            }
        }
        jacobian = jacobian_new;

        for(unsigned int i = 0; i < cartesian_wrenches_.size(); ++i)
        {
            for(unsigned int j = 0; j < 6; ++j)
            {
                jacobian(link_start_index + j, i) = partial_jacobian(j, i);
            }
        }

    }
}

void Tree::fillCartesianWrench(Eigen::VectorXd& all_wrenches) {
    unsigned int link_start_index = all_wrenches.rows();

    // Add 6 new values to current whole body wrench vector (TODO: could be much more efficient)

    Eigen::VectorXd wrench_new(all_wrenches.rows() + 6 * cartesian_wrenches_.size());
    wrench_new.setZero();

    for(unsigned int i = 0; i < all_wrenches.rows(); ++i) {
        wrench_new(i) = all_wrenches(i);
    }
    all_wrenches = wrench_new;

    // Fill all_wrenches with the wrenches added to this chain

    for(std::map<std::string, Eigen::VectorXd>::iterator it_wrench = cartesian_wrenches_.begin(); it_wrench != cartesian_wrenches_.end(); ++it_wrench)
    {
        for(unsigned int i = 0; i < 6; ++i)
        {
            all_wrenches(i + link_start_index) = it_wrench->second(i);
        }
        link_start_index += 6;
    }

    //cout << "torque = " << endl;
    //cout << torque << endl;
}

void Tree::addCartesianWrench(const std::string& link_name, const Eigen::VectorXd& wrench)
{
    std::map<std::string, Eigen::VectorXd>::iterator it_wrench = cartesian_wrenches_.find(link_name);
    if (it_wrench == cartesian_wrenches_.end())
    {
        cartesian_wrenches_[link_name] = wrench;
    }
    else
    {
        it_wrench->second += wrench;
    }
    //std::cout << cartesian_wrenches_.size() << std::endl;
}

void Tree::removeCartesianWrenches() {
    cartesian_wrenches_.clear();
}

void Tree::calcPartialJacobian(std::string& link_name,
                               KDL::JntArray& q_in,
                               Eigen::MatrixXd jacobian)
{
    KDL::Jacobian kdl_jacobian;
    KDL::JntArray q_tree;
    kdl_jacobian.resize(kdl_tree_.getNrOfJoints());
    q_tree.resize(kdl_tree_.getNrOfJoints());
    //std::cout << q_tree.data.size() << std::endl;
    //std::cout << tree_joint_index_.size() << std::endl;
    uint i = 0;
    for(std::vector<int>::iterator it_index = tree_joint_index_.begin(); it_index != tree_joint_index_.end(); ++it_index, ++i)
    {
        int index = *it_index;
        if (index < 0 )
        {
            q_tree.data[i] = 1;
        }
        else if (index >= 0 )
        {
            q_tree.data[i] = q_in.data[index];
        }
    }

    jac_solver_->JntToJac(q_in,kdl_jacobian,link_name);

    //std::cout << kdl_jacobian.rows() << std::endl;
    //std::cout << kdl_jacobian.columns() << std::endl;

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

    Eigen::MatrixXd printJ(kdl_jacobian.rows(),kdl_jacobian.columns());
    for(unsigned int i = 0; i < kdl_jacobian.rows(); ++i)
    {
        for(unsigned int j = 0; j < kdl_jacobian.columns(); ++j)
        {
            printJ(i, j) = kdl_jacobian(i, j);
        }
    }
    //cout << "link_name = " << link_name << std::endl;
    //cout << "printJ = " << endl << printJ << endl;
    //cout << "jacobian = " << endl << jacobian << endl;
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

    for(std::map<std::string,KDL::TreeElement>::const_iterator it_segment = segments_map.begin(); it_segment != segments_map.end(); ++it_segment)
    {
        std::pair<std::string,KDL::TreeElement> pair = *it_segment;
        std::map<std::string,KDL::TreeElement>::const_iterator segment_itr = tree.getSegment(pair.first);
        std::pair<std::string,KDL::TreeElement> segment = *segment_itr;

        //std::cout << segment.second.segment.getName() << std::endl;

        const KDL::Joint joint = segment.second.segment.getJoint();

        if (joint.getType() != KDL::Joint::None)
        {
            std::map<std::string, unsigned int>::iterator it_names = joint_name_to_index_.find(joint.getName());
            if (it_names == joint_name_to_index_.end())
            {
                tree_joint_index.push_back(-1);
                std::cout << joint.getName() << ": " << -1 << std::endl;
            }
            else
            {
                std::pair<std::string, unsigned int> name = *it_names;
                tree_joint_index.push_back(name.second);
                std::cout << joint.getName() << ": " << name.second << std::endl;
            }
        }
    }
}
