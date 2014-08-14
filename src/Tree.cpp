#include "Tree.h"

Tree::Tree() {
}

Tree::~Tree() {
}

void Tree::calcPartialJacobian(const std::string& link_name,
                               Eigen::MatrixXd& jacobian)
{
    KDL::Jacobian kdl_jacobian;
    kdl_jacobian.resize(kdl_tree_.getNrOfJoints());

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

void Tree::getJointNames(std::map<std::string, unsigned int> &jnt_name_to_index_in,std::map<std::string, unsigned int> &jnt_name_to_index_out)
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

unsigned int Tree::getNrJoints()
{
    return number_of_joints_;
}
