#include "bayesianFilter.h"
#include <iostream>
#include <algorithm>

using namespace std;

//constructor:
bayesianFilter::bayesianFilter()
{
    //set initialization to false:
    is_initialized_ = false;

    //set standard deviation of control:
    control_std     = 1.0f;

    //set standard deviation of observations:
    observation_std = 1.0f;
    
    //define size of different state vectors:
    bel_x.resize(100,0);
    bel_x_init.resize(100,0);
}


void bayesianFilter::process_measurement(const MeasurementPackage &measurements,
                                         const map &map_1d,
                                         help_functions &helpers)
{
    /******************************************************************************
     *  Set init belief of state vector:
     ******************************************************************************/
    if(!is_initialized_)
    {
        //run over map:
        for (int i = 0; i < map_1d.landmark_list.size(); ++i)
        {
            //define landmark:
            map::single_landmark_s landmark_temp;
            //get landmark from map:
            landmark_temp = map_1d.landmark_list[i];

            //check, if landmark position is in the range of state vector x:
            if(landmark_temp.x_f > 0 && landmark_temp.x_f < bel_x_init.size())
            {
                //cast float to int:
                int position_x = int(landmark_temp.x_f) ;
                //set belief to 1:
                bel_x_init[position_x]     = 1.0f;
                bel_x_init[position_x - 1] = 1.0f;
                bel_x_init[position_x + 1] = 1.0f;
            }
        }

        //normalize belief at time 0:
        bel_x_init = helpers.normalize_vector(bel_x_init);

        //set initial flag to true:
        is_initialized_ = true;
    }


    /******************************************************************************
     *  motion model and observation update
    ******************************************************************************/
    cout <<"-->motion model for state x ! \n" << endl;

    //get current observations and control information:
    MeasurementPackage::control_s     controls = measurements.control_s_;
    MeasurementPackage::observation_s observations = measurements.observation_s_;

    //run over the whole state (index represents the pose in x!):
    for (int i = 0; i < bel_x.size(); ++i)
    {
        float pose_i = float(i);
        /**************************************************************************
         *  posterior for motion model
        **************************************************************************/

        // motion posterior:
        float posterior_motion = 0.0f;

        //loop over state space x_t-1 (convolution):
        for (int j=0; j< bel_x.size(); ++j)
        {
            float pose_j = float(j) ;    
            
            float distance_ij = pose_i-pose_j;

            //transition probabilities:
            float transition_prob = helpers.normpdf(distance_ij,
                                                    controls.delta_x_f,
                                                    control_std) ;
            //motion model:
            posterior_motion += transition_prob*bel_x_init[j];
        }

        /**************************************************************************
         *  observation update:
        **************************************************************************/
            
        //define pseudo observation vector:
        vector<float> pseudo_ranges;

        //define maximum distance:
        float distance_max = 100;
            
        //loop over number of landmarks and estimate pseudo ranges:
        for (int l = 0; l < map_1d.landmark_list.size(); ++l)
        {
            //estimate pseudo range for each single landmark 
            //and the current state position pose_i:
            float range_l = map_1d.landmark_list[l].x_f - pose_i;
            
            //check, if distances are positive: 
            if(range_l > 0.0f)
                pseudo_ranges.push_back(range_l);
        }

        //sort pseudo range vector:
        sort(pseudo_ranges.begin(), pseudo_ranges.end());

        //define observation posterior:
        float posterior_obs = 1.0f;
        
        //run over current observation vector:
        for (int z = 0; z < observations.distance_f.size(); ++z)
        {
            //define min distance:
            float pseudo_range_min;

            //check, if distance vector exists:
            if(pseudo_ranges.size() > 0)
            {
                //set min distance:
                pseudo_range_min = pseudo_ranges[0];
                //remove this entry from pseudo_ranges-vector:
                pseudo_ranges.erase(pseudo_ranges.begin());

            }
            //no or negative distances: set min distance to maximum distance:
            else
                pseudo_range_min = distance_max ;

            //estimate the posterior for observation model: 
            posterior_obs *= helpers.normpdf(observations.distance_f[z], 
                                             pseudo_range_min,
                                             observation_std); 
        }
        
        /**************************************************************************
         *  finalize bayesian localization filter:
         *************************************************************************/
        
        //update = observation_update* motion_model
        bel_x[i] = posterior_obs*posterior_motion;

    }; 

    //normalize:
    bel_x = helpers.normalize_vector(bel_x);

    //set bel_x to belief_init:
    bel_x_init = bel_x;
};
