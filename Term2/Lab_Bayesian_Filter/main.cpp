#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

#include "measurement_package.h"
#include "map.h"
#include "help_functions.h"
#include "bayesianFilter.h"

using namespace std;

int main()
{
    /******************************************************************************
     *  declaration:                                                                  
     *****************************************************************************/
    
    //define example: 01, 02, 03, 04
    string example_string = "04";

    //declare map:
    map map_1d;

    //declare measurement list:
    vector<MeasurementPackage> measurement_pack_list;

    //declare helpers:
    help_functions helper;

    //define file names:
    char in_file_name_ctr[1024];
    char in_file_name_obs[1024];
    char in_file_name_gt[1024];

    /******************************************************************************
     *  read map and measurements:                                               *
     *****************************************************************************/
    //read map:
    helper.read_map_data("data/map_1d.txt", map_1d);

    //define file name of controls:
    sprintf(in_file_name_ctr, "data/example%s/control_data.txt", 
            example_string.c_str());

    //define file name of observations:
    sprintf(in_file_name_obs, "data/example%s/observations/", 
            example_string.c_str());
    
    //read in data to measurement package list:
    helper.read_measurement_data(in_file_name_ctr, 
                                 in_file_name_obs, 
                                 measurement_pack_list);

    /*******************************************************************************
     *  start 1d_bayesian filter                                                   *
     *******************************************************************************/

    //create instance of 1d_bayesian localization filter:
    bayesianFilter localization_1d_bayesian;

    //cycle:
    for (size_t t = 0; t < measurement_pack_list.size(); ++t)
        //Call 1d_bayesian filter:
        localization_1d_bayesian.process_measurement(measurement_pack_list[t],
                                                     map_1d,
                                                     helper);

    /*******************************************************************************
     *  print/compare results:                                                 *
     ********************************************************************************/
    
    //define file name of gt data:
    sprintf(in_file_name_gt, "data/example%s/gt_example%s.txt",example_string.c_str(),example_string.c_str() );

    ///compare gt data with results:
    helper.compare_data(in_file_name_gt, localization_1d_bayesian.bel_x);
    

    return 0;
}