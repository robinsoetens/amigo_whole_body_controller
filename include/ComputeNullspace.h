/*!
 * \author Janno Lunenburg
 * \date October, 2012
 * \version 0.1
 */

#ifndef COMPUTENULLSPACE_H_
#define COMPUTENULLSPACE_H_

// Eigen
#include <Eigen/Core>

class ComputeNullspace {

public:

    /**
     * Constructor
     */
    ComputeNullspace();

    /**
     * Deconstructor
     */
    virtual ~ComputeNullspace();

    /*
     * Initialize function required for resizing etc
     */
    void initialize(const uint num_joints, const Eigen::MatrixXd& A);

    /*
     * Update function
     */
    void update(const Eigen::MatrixXd& J, Eigen::MatrixXd& N);



protected:

    //! Weighting Matrix
    Eigen::MatrixXd A_;

    //! Unity Matrix
    Eigen::MatrixXd I_;

    //! Weighted pseudo-inverse
    ///Eigen::MatrixXd Jpinv_;

    //! Matrix containing inverted singular values
    Eigen::MatrixXd Sinv_;

    //! Number of DoFs active during previous update
    uint previous_num_active_dofs_;


};


#endif
