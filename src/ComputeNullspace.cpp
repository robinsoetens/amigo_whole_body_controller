#include "ComputeNullspace.h"
#include <Eigen/SVD>
#include <ros/ros.h>

ComputeNullspace::ComputeNullspace() {

}

ComputeNullspace::~ComputeNullspace() {

}

void ComputeNullspace::initialize(const uint num_dofs, const uint num_joints, const Eigen::MatrixXd& A) {

    ///Jpinv_.resize(num_joints,num_dof);
    A_.resize(num_joints,num_joints);
    A_ = A;
    Sinv_.resize(num_dofs,num_dofs);
    for (uint i = 0; i < num_dofs; i++) {
        for (uint j = 0; j < num_dofs; j++) {
            Sinv_(i,j) = 0;
        }
    }

    I_.resize(num_joints,num_joints);
    for (uint i = 0; i < num_joints; i++) {
        for (uint j = 0; j < num_joints; j++) {
            if (i == j) I_(i,j) = 1;
            else I_(i,j) = 0;
        }
    }
    ROS_INFO("Nullspace calculator initialized");

}

void ComputeNullspace::update(const Eigen::MatrixXd& J, Eigen::MatrixXd& N) {

    // See Dietrich 2011

    Eigen::MatrixXd WJ = J*A_*J.transpose();

    //Eigen::JacobiSVD<Eigen::MatrixXd> WJsvd(WJ, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd> WJsvd(WJ, Eigen::ComputeThinU | Eigen::ComputeFullV);

    ROS_INFO("Computing nullspace");

    // ToDO: Choose eps wisely
    double eps=0.00001;
    Eigen::VectorXd Svec = WJsvd.singularValues();
    for (int i = 0; i < WJsvd.singularValues().rows(); i++) {
        if (Svec(i) > eps) Sinv_(i,i) = 1.0/Svec(i);
        else Sinv_(i,i) = 0;
    }
    ROS_INFO("Sigma inverse computed");

    ROS_INFO("Dimensions of A:  %i, %i", A_.rows(), A_.cols());
    ROS_INFO("Dimensions of Jt: %i, %i", J.cols(), J.rows());
    ROS_INFO("Dimensions of V:  %i, %i", WJsvd.matrixV().rows(), WJsvd.matrixV().cols());
    ROS_INFO("Dimensions of Si: %i, %i", Sinv_.rows(), Sinv_.cols());
    ROS_INFO("Dimensions of Ut: %i, %i", WJsvd.matrixU().cols(), WJsvd.matrixV().rows());

    // Jpinv = A*J^T * (J*A*J^T)^(-1)
    Eigen::MatrixXd Jpinv = A_ * J.transpose() * WJsvd.matrixV() * Sinv_ * WJsvd.matrixU().transpose();

    ROS_INFO("Jpinv computed");

    N = I_ - J.transpose() * Jpinv.transpose();

    ROS_INFO("N computed");

}
