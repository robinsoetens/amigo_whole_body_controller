/*!
 * \author Janno Lunenburg
 * \date September, 2012
 * \version 0.1
 */

#ifndef WBC_OBSTACLEAVOIDANCE_H_
#define WBC_OBSTACLEAVOIDANCE_H_

// ROS
#include "ros/ros.h"

// tf
#include <tf/transform_listener.h>

// Eigen
#include <Eigen/Core>

/////

class ObstacleAvoidance {

    struct Box {

        Box(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
            : min_(min), max_(max) {
        }

        Eigen::Vector3d min_;
        Eigen::Vector3d max_;
    };

public:

    //ToDo: make configure, start- and stophook. Components can be started/stopped in an actionlib kind of fashion

    /**
     * Constructor
     */
    ObstacleAvoidance();

    /**
     * Deconstructor
     */
    virtual ~ObstacleAvoidance();

    /**
     * Initialize function
     */
    bool initialize(const std::string&, uint);

    /**
     * Update
     */
    void update(Eigen::VectorXd&, uint& force_vector_index);

    void visualize(const Eigen::Vector3d& dir_position) const;

protected:

    std::vector<Box*> boxes_;

    tf::TransformListener listener_;

    std::string end_effector_frame_;  

    uint F_start_index_;

    ros::Publisher pub_marker_;

};

#endif
