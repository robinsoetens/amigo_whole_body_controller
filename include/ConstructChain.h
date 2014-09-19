/*!
 * \author Janno Lunenburg
 * \date September, 2012
 * \version 0.1
 */

#ifndef CONSTRUCT_CHAIN_H_
#define CONSTRUCT_CHAIN_H_

#include <urdf/model.h>

// KDL
#include <kdl_parser/kdl_parser.hpp>
//#include <kdl/jntarray.hpp>
#include <kdl/treejnttojacsolver.hpp>

// Boost
#include <boost/scoped_ptr.hpp>

// Print using cout statements
#include <iostream>

class ConstructChain {
public:

    bool error;

    unsigned int num_joints;

    /*
     * Constructor
     */
    ConstructChain();

    /*
     * Deconstructor
     */
    virtual ~ConstructChain();

    /*
     * Initialize tree and solvers
     */
    bool initialize();

    KDL::Jacobian getJacobian(const KDL::JntArray);

    //KDL::Tree tree_;

private:


    //KDL::Tree* tree_;
    //KDL::TreeJntToJacSolver* treeJntToJacSolver_;
    boost::scoped_ptr<KDL::TreeJntToJacSolver> treeJntToJacSolver_;

    bool loadModel(const std::string xml);
    bool readJoints(urdf::Model &robot_model);
    bool readLinks(urdf::Model &robot_model);
    bool constructJacobianSolver();

};

#endif
