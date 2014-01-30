/*!
 * \author Janno Lunenburg
 * \date December, 2013
 * \version 0.1
 */

#ifndef WBC_TRACING_H_
#define WBC_TRACING_H_

#include <string>
#include <vector>
#include <Eigen/Core>
#include <kdl/frames.hpp>

class Tracing {

public:

    /** Constructor */
    Tracing();

    /** Deconstructor */
    ~Tracing();

    /** Initialization function
      * @param filename: Path + file name
      * @param column_names: vector with strings containing the column names
      * @param buffer_length: length of tracing
      */
    bool Initialize(const std::string& foldername, const std::string &filename, const std::vector<std::string> &column_names, unsigned int buffer_length);

    /** Writes all relevant data to the tracing buffer
      * with various data types
      * @param start_index: index of first column where data should be inserted */
    void collectTracing(unsigned int start_index, const double& data);
    void collectTracing(unsigned int start_index, const std::vector<double>& data);
    void collectTracing(unsigned int start_index, const Eigen::VectorXd& data);
    void collectTracing(unsigned int start_index, const KDL::Frame& data);
    void collectTracing(unsigned int start_index, const KDL::Twist& data);
    void collectTracing(unsigned int start_index, const KDL::Wrench& data);

    /** Sets buffer to next line (increases buffer_index_)
      * There should always be a newLine() before tracing starts */
    void newLine();

    /** Writes the buffer to a textfile
      * Will be called when buffer is full but can also be called externally */
    void writeToFile();

protected:

    /** String containing the filename
      * This should include the entire path! */
    std::string filename_;

    /** Matrix that will be filled for tracing
      * Every vector of doubles contains a ROW of the matrix for easy indexing */
    std::vector<std::vector<double> > buffers_;

    /** Vector containing the column names */
    std::vector<std::string> column_names_;

    /** Buffer length, number of columns and bufferindex */
    unsigned int number_columns_, buffer_length_;
    int buffer_index_;

    // ToDo: remove
    bool wroteToFile_;

};

#endif
