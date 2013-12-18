#include "amigo_whole_body_controller/Tracing.hpp"

/// For tracing
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <time.h>

Tracing::Tracing() :
    number_columns_(0),
    buffer_length_(0) {

}


Tracing::~Tracing() {

    /// Write data to file upon deconstruction
    writeToFile();

}

bool Tracing::Initialize(std::string& filename, const std::vector<std::string>& column_names, unsigned int buffer_length) {

    /// Add timestamp to filename
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];
    time (&rawtime);
    timeinfo = localtime (&rawtime);
    strftime (buffer,80,"_%Y%m%d%H%M%S.dat",timeinfo);
    std::string fileext(buffer);
    filename_ = filename + fileext;

    column_names_ = column_names;
    number_columns_ = column_names.size()+1; /// Number of columns + time column
    buffer_length_ = buffer_length;
    buffers_.resize(buffer_length);

    /// Pre-allocate memory and assign zeros
    for (unsigned int i = 0; i < buffer_length_; i++) {
        buffers_[i].assign(number_columns_, 0.0);
    }

    buffer_index_ = -1;
    wroteToFile_ = false;

    return true;
}

void Tracing::collectTracing(unsigned int start_index, const double& data) {

    /// Only perform if data fits
    if (start_index <= number_columns_ && buffer_index_ < (int)buffer_length_) {
        buffers_[buffer_index_][start_index] = data;
    }
}

void Tracing::collectTracing(unsigned int start_index, const std::vector<double>& data) {

    /// Only perform if data fits
    if (start_index + data.size() <= number_columns_ && buffer_index_ < (int)buffer_length_) {

        /// Data
        for (unsigned int i = 0; i < data.size(); i++) {
            buffers_[buffer_index_][i+start_index] = data[i];
        }
    }
}

void Tracing::collectTracing(unsigned int start_index, const Eigen::VectorXd& data) {

    /// Only perform if data fits
    if (start_index + data.rows() <= number_columns_ && buffer_index_ < (int)buffer_length_) {

        /// Data
        for (unsigned int i = 0; i < data.rows(); i++) {
            buffers_[buffer_index_][i+start_index] = data(i);
        }
    }
}

void Tracing::collectTracing(unsigned int start_index, const KDL::Frame& data) {

    /// Only perform if data fits
    if (start_index + 6 <= number_columns_ && buffer_index_ < (int)buffer_length_) {

        /// Data
        buffers_[buffer_index_][start_index] = data.p.x();
        buffers_[buffer_index_][1+start_index] = data.p.y();
        buffers_[buffer_index_][2+start_index] = data.p.z();
        double roll, pitch, yaw;
        data.M.GetRPY(roll, pitch, yaw);
        buffers_[buffer_index_][3+start_index] = roll;
        buffers_[buffer_index_][4+start_index] = pitch;
        buffers_[buffer_index_][5+start_index] = yaw;
    }
}

void Tracing::collectTracing(unsigned int start_index, const KDL::Twist &data) {

    /// Only perform if data fits
    if (start_index + 6 <= number_columns_ && buffer_index_ < (int)buffer_length_) {

        /// Data
        buffers_[buffer_index_][start_index] = data.vel.x();
        buffers_[buffer_index_][1+start_index] = data.vel.y();
        buffers_[buffer_index_][2+start_index] = data.vel.z();
        buffers_[buffer_index_][3+start_index] = data.rot.x();
        buffers_[buffer_index_][4+start_index] = data.rot.y();
        buffers_[buffer_index_][5+start_index] = data.rot.z();
    }
}

void Tracing::collectTracing(unsigned int start_index, const KDL::Wrench& data) {

    /// Only perform if data fits
    if (start_index + 6 <= number_columns_ && buffer_index_ < (int)buffer_length_) {

        /// Data
        buffers_[buffer_index_][start_index] = data.force.x();
        buffers_[buffer_index_][1+start_index] = data.force.y();
        buffers_[buffer_index_][2+start_index] = data.force.z();
        buffers_[buffer_index_][3+start_index] = data.torque.x();
        buffers_[buffer_index_][4+start_index] = data.torque.y();
        buffers_[buffer_index_][5+start_index] = data.torque.z();
    }
}

void Tracing::newLine() {

    /// Only do when buffer is not yet full
    if (buffer_index_ < int(buffer_length_)) {

        /// Increase buffer index
        ++buffer_index_;

        /// Time
        if (buffer_index_ >= 0 && buffer_index_ < (int)buffer_length_) {
            double stamp = ros::Time::now().toSec();
            buffers_[buffer_index_][0] = stamp;
        }

    } else {
        if (!wroteToFile_) {
            ROS_WARN_ONCE("Buffer of length %i is full", (int)buffer_length_);
            ros::Time start = ros::Time::now();
            writeToFile();
            ros::Time end = ros::Time::now();
            double duration = (end-start).toSec();
            ROS_WARN_ONCE("Writing took %f seconds", duration);
        }
        // ToDo: write to file upon deconstruction
        //} else {
        //    ROS_WARN_ONCE("Buffer of length %i is full", (int)buffer_length_);
        //}
    }
}

void Tracing::writeToFile() {

    ROS_INFO("Write to file: %s", filename_.c_str());
    //ROS_INFO("Number of rows and columns = %i, %i", (int)buffers_.size(), (int)buffers_[0].size());

    /// Only write if buffer is not empty
    if (buffers_.size() > 0) {
        ROS_INFO("Write data to file, size = %i", (int)buffers_.size());

        /// Take value of first stamp such that time starts counting from 0
        double first_stamp = buffers_[0][0];

        /// Open file
        std::ofstream my_file;
        my_file.open(filename_.c_str());

        /// Fill header
        my_file << "Time";
        for (unsigned int i = 1; i < number_columns_; i++) {
            my_file << "\t" << column_names_[i-1];
        }
        my_file << "\n";

        /// Loop over rows
        for (unsigned int i = 0; i < buffers_.size(); i++) {
            //ROS_INFO("Row %i", (int)i);
            my_file << buffers_[i][0] - first_stamp;

            /// Loop over columns
            for (unsigned int j = 1; j < number_columns_; j++) {
                //ROS_INFO("column %i", (int)j);
                my_file << "\t" << buffers_[i][j];
            }
            my_file << "\n";
        }
        my_file.close();
    }
    wroteToFile_ = true;
}
