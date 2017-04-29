#ifndef HELP_FUNCTIONS_H_
#define HELP_FUNCTIONS_H_

#include <math.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "measurement_package.h"


class help_functions
{
public:

    //definition of one over square root of 2*pi:
    float ONE_OVER_SQRT_2PI = 1/sqrt(2*M_PI);

    //definition square:
    inline float squared(float x)
    {
        return x*x;
    }

    /*****************************************************************************
     * normpdf(X,mu,sigma) computes the probability function at values x using the
     * normal distribution with mean mu and standard deviation std. x, mue and 
     * sigma must be scalar! The parameter std must be positive. 
     * The normal pdf is y=f(x;mu,std)= 1/(std*sqrt(2pi)) e[ -(x−mu)^2 / 2*std^2 ]
    *****************************************************************************/
    inline float normpdf(float x, float mu, float std)
    {
        return (ONE_OVER_SQRT_2PI/std)*exp(-0.5*squared((x-mu)/std));
    }

    //function to normalize a vector:
    inline std::vector<float> normalize_vector(std::vector<float> inputVector)
    {
        //declare sum:
        float sum = 0.0f;

        //declare and resize output vector:
        std::vector<float> outputVector ;
        outputVector.resize(inputVector.size());

        //estimate the sum:
        for (int i = 0; i < inputVector.size(); ++i) {
            sum += inputVector[i];
        }

        //normalize with sum:
        for (int i = 0; i < inputVector.size(); ++i) {
            outputVector[i] = inputVector[i]/sum;
        }

        //return normalized vector:
        return outputVector;
    }


    /* Reads map data from a file.
     * @param filename Name of file containing map data.
     */
    inline bool read_map_data(std::string filename, map& map) 
    {
        // Get file of map:
        std::ifstream in_file_map(filename.c_str(),std::ifstream::in);

        // Return if we can't open the file.
        if (!in_file_map)   return false;

        //declare single line of map file:
        std::string line_map;

        //run over each single line:
        while(getline(in_file_map, line_map))
        {
            std::istringstream iss_map(line_map);

            //declare landmark values and ID:
            float landmark_x_f;
            int id_i;

            //read data from current line to values::
            iss_map >> id_i ;
            iss_map >> landmark_x_f ;

            //declare single_landmark:
            map::single_landmark_s single_landmark_temp ;

            //set values
            single_landmark_temp.id_i = id_i ;
            single_landmark_temp.x_f  = landmark_x_f;

            //push_back in landmark list of map_1d:
            map.landmark_list.push_back(single_landmark_temp);

        }
        return true;
    }


    /* Reads measurements from a file.
     * @param filename Name of file containing measurement  data.
     */
    inline bool read_measurement_data(std::string filename_control,
                                      std::string filename_obs,
                                      std::vector<MeasurementPackage>& measurement_pack_list)
    {
        //get file of measurements:
        std::ifstream in_file_control(filename_control.c_str(),std::ifstream::in);

        if (!in_file_control)   return false;

        //declare single line of measurement file:
        std::string line;

        int count = 1;

        //run over each single line:
        while(getline(in_file_control, line))
        {
            //declare measurement package:
            MeasurementPackage meas_package;

            std::istringstream iss(line);

            //declare position values:
            float delta_x_f;

            //read data from line to values:
            iss >> delta_x_f;

            //set control information:
            meas_package.control_s_.delta_x_f = delta_x_f ;

            //read observations for each control information:
            char str_obs[1024];

            //define file name of observations for current control/position info:
            sprintf(str_obs,"%sobservations_%06i.txt", filename_obs.c_str(), count);
            std::string in_file_name_observation = std::string(str_obs);


            //get file of observations:
            std::ifstream in_file_observation(in_file_name_observation.c_str(),
                                              std::ifstream::in);
            if (!in_file_observation)   return false;

            std::string line_obs;

            //run over each single line:
            while(getline(in_file_observation, line_obs))
            {
                std::istringstream iss_obs(line_obs);

                //declare observation values:
                float distance_f;

                //read data from line to values:
                iss_obs >> distance_f;

                //set observation information:
                meas_package.observation_s_.distance_f.push_back(distance_f);
            }
            //push_back single package in measurement list:
            measurement_pack_list.push_back(meas_package);

            //increase counter for observation files:
            count++;
        }
        return true;
    }

    inline bool compare_data(std::string filename_gt,
                             std::vector<float>& result_vec)
    {
        /*****************************************************************************
         *  print/compare results:                                                 *
         *****************************************************************************/
        //get GT data:
        //define file name of map:

        std::vector<float> gt_vec;

        //get file of map:
        std::ifstream in_file_gt(filename_gt.c_str(),std::ifstream::in);

        //declare single line of map file:
        std::string line_gt;

        //run over each single line:
        while(getline(in_file_gt, line_gt))
        {
            std::istringstream iss_gt(line_gt);
            float gt_value;

            //read data from current line to values::
            iss_gt >> gt_value  ;
            gt_vec.push_back(gt_value);

        }

        float error_sum = 0.0, belief_sum = 0.0;
        std::cout <<"..................................................."<< std::endl;
        std::cout <<"...............----> Results <----................."<< std::endl;
        std::cout <<"..................................................."<< std::endl;

        for (int i = 0; i <  result_vec.size(); ++i)
        {
            error_sum+= (gt_vec[i]-result_vec[i])*(gt_vec[i]-result_vec[i]);
            belief_sum+= result_vec[i] ;

            std::cout << std::fixed << std::setprecision(5) <<"bel_x="<< i <<":" << "\t"
                      << result_vec[i]<<"\t"
                      << "ground truth:" << "\t"
                      << gt_vec[i]<<"\t" << std::endl ;
        }

        std::cout <<"..................................................."<< std::endl;
        std::cout << std::fixed << std::setprecision(5)<< "sum bel:"    << "\t" << belief_sum <<std::endl;
        std::cout <<"..................................................."<< std::endl;
        std::cout << std::fixed << std::setprecision(5)<< " rse   :     "<< "\t" << sqrt((error_sum)) <<std::endl;
        std::cout <<"..................................................."<< std::endl;
        std::cout <<"..................................................."<< std::endl;

        return true;
    }

};

#endif /* HELP_FUNCTIONS_H_ */
