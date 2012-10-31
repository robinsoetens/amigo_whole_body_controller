#include "ComputeJacobian.h"

ComputeJacobian::ComputeJacobian() {

}

ComputeJacobian::~ComputeJacobian() {

}

void ComputeJacobian::Update(const KDL::JntArray& q_current, const std::map<std::string, Component*> component_map, Eigen::MatrixXd& Jacobian) {

    Jacobian.setZero();

    KDL::JntArray chain_joint_array;
    KDL::Jacobian chain_jacobian;
   // uint chain_index = 0;

    // Compute Jacobian for every chain
    for (uint i = 0; i < chain_description_array.size(); i++) {

        printf("\nChain description: ");
        for (uint j = 0; j < chain_description_array[i].size(); j++) {
            printf("%s ", chain_description_array[i][j].c_str());
        }
        printf("\n");

        ///ROS_INFO("Chain description array %i = %d",i,isactive_vector[i]);

        // Only update the Jacobian for chains with an active task
        if (isactive_vector[i]) {

            printf("    ACTIVE\n");

            ///index = 0;
            // Fill in correct joint array
            // ToDo: RESIZING SHOULDN'T BE DONE IN REALTIME
            chain_joint_array.resize(chain_array[i].getNrOfJoints());
            chain_jacobian.resize(chain_array[i].getNrOfJoints());
            ///ROS_INFO("Number of joints of this chain = %i", chain_array[i].getNrOfJoints());
            uint current_index = chain_array[i].getNrOfJoints();

            // Fill in joints for every component in this chain
            for (uint ii = 0; ii < chain_description_array[i].size(); ii++) {

                std::string component_name = chain_description_array[i][ii];
                ///ROS_INFO("Inserting %i joint positions of %s", component_description_map[component_name].number_of_joints, component_name.c_str());
                ///ROS_INFO("Size of jointarray of %s = %i", component_name.c_str(), component_description_map[component_name].q.size());

                // For every joint within this component
                // ToDo: make this more effective

                ///ROS_INFO("First value is %f", component_description_map[component_name].q[0]);

                const component_description& component = component_description_map.find(component_name)->second;

                current_index -= component.number_of_joints;
                for (int iii = 0; iii< component.number_of_joints; iii++) {
                    ///chain_joint_array(current_index+iii) = component_description_map[component_name].q[iii];
                    const std::string& joint_name = component.joint_names[iii];
                    chain_joint_array(current_index+iii) = q_current(joint_name_index_map[joint_name]);
                }

            }

            printf("%i Size: %i\n", i, chain_joint_array.rows());
            for(int unsigned k = 0; k < chain_joint_array.rows(); ++k) {
                printf("%f ", chain_joint_array(k));
            }
            printf("\n");

            ///ROS_INFO("Chain %s",chain_description_array[i][0].c_str());
            ///ROS_INFO("Joint values: %f, %f, %f, %f, %f, %f, %f, %f",chain_joint_array(0),chain_joint_array(1),chain_joint_array(2),chain_joint_array(3),chain_joint_array(4),chain_joint_array(5),chain_joint_array(6),chain_joint_array(7));

            // Solve Jacobian of chain
            jnt_to_jac_solver_array[i]->JntToJac(chain_joint_array,chain_jacobian);

            ///uint show_column = 0;
            ///ROS_INFO("%i column of computed Jacobian is \n%f\n%f\n%f\n%f\n%f\n%f",show_column+1,chain_jacobian.data(0,show_column),chain_jacobian.data(1,show_column),chain_jacobian.data(2,show_column),chain_jacobian.data(3,show_column),chain_jacobian.data(4,show_column),chain_jacobian.data(5,show_column));

            // Put resulting Jacobian in correct blocks into large Jacobian
            ///Block of size (p,q), starting at (i,j)   matrix.block(i,j,p,q);
            // For every component in the chain
            ///uint current_index = 0;
            current_index = chain_array[i].getNrOfJoints();
            for (uint iiii = 0; iiii < chain_description_array[i].size(); iiii++) {
                std::string component_name = chain_description_array[i][iiii];
                const component_description& component = component_description_map.find(component_name)->second;

                current_index -= component.number_of_joints;

                printf("%i ", current_index);

                Jacobian.block(6 * i, component.start_index, 6, component.number_of_joints) =
                        chain_jacobian.data.block(0, current_index, 6, component.number_of_joints);
                ///current_index += component_description_map[component_name].number_of_joints;

               // ROS_INFO("Size of Jacobian is %i x %i",Jacobian.data.rows(),Jacobian.data.cols());
            }
            printf("\n");

        } else {
            printf("    NOT ACTIVE\n");
        }

        ///ROS_INFO("Eighth column of computed Jacobian is \n%f\n%f\n%f\n%f\n%f\n%f",Jacobian(0,7),Jacobian(1,7),Jacobian(2,7),Jacobian(3,7),Jacobian(4,7),Jacobian(5,7));
    }

}
