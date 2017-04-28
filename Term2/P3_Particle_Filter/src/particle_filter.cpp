#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <limits>
#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, 
                          double y, 
                          double theta, 
                          double std[]) 
{
    // Number of particles
    num_particles = 1000;

    // Random number generator
    default_random_engine gen;

    // Normal distributions for initial coordinates
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    // Initialize the particles and their weights
    particles.resize(num_particles);
    weights.resize(num_particles);
    const double weight = 1.0; 
    for (int i = 0; i < num_particles; ++i)
    {
        const double p_x = dist_x(gen);
        const double p_y = dist_y(gen);
        const double p_theta = dist_theta(gen);
        particles[i] = {i, p_x, p_y, p_theta, weight};
        weights[i] = weight;
    }

    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t,
                                double std_pos[], 
                                double velocity, 
                                double yaw_rate)
{
    // Random number generator
    std::default_random_engine gen;

    // Normal distributions for new coordinates
    std::normal_distribution<double> dist_x(0, std_pos[0]);
    std::normal_distribution<double> dist_y(0, std_pos[1]);
    std::normal_distribution<double> dist_theta(0, std_pos[2]);

    for (Particle & particle: particles)
    {   
        if (abs(yaw_rate) < 1.0e-5)
        {
            particle.x += velocity * delta_t * cos(particle.theta);
            particle.y += velocity * delta_t * sin(particle.theta);
        }
        else
        {
            particle.x += velocity / yaw_rate * (sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta));
            particle.y += velocity / yaw_rate * (cos(particle.theta) - cos(particle.theta + yaw_rate * delta_t));
        }
        particle.theta = particle.theta + yaw_rate * delta_t;

        particle.x += dist_x(gen);
        particle.y += dist_y(gen);
        particle.theta += dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations)
{
    for (LandmarkObs & obs : observations)
    {
        double min_distance = numeric_limits<double>::max();
        int min_id;
        for(const LandmarkObs & pred : predicted)
        {
            const double distance = dist(obs.x, obs.y, pred.x, pred.y);
            if(distance < min_distance)
            {
                min_distance = distance;
                min_id = pred.id;
            }
        }
        obs.id = min_id;
    }
}

void ParticleFilter::updateWeights(double sensor_range, 
                                   double std_landmark[], 
                                   vector<LandmarkObs> observations, 
                                   Map map_landmarks)
{
    for (int i = 0; i < num_particles; ++i)
    {    
        // Define observations in world frame (map's coordinate)
        vector<LandmarkObs> world_obs(observations.size());
        
        // Convert observations from local frame to world frame
        for (int j = 0; j < observations.size(); ++j)
        {
            LandmarkObs landmark;
            landmark.id = observations[j].id;
            landmark.x = observations[j].x * cos(particles[i].theta) - 
                         observations[j].y * sin(particles[i].theta) + particles[i].x;
            landmark.y = observations[j].x * sin(particles[i].theta) + 
                         observations[j].y * cos(particles[i].theta) + particles[i].y;
            world_obs[j] = landmark;
        }
    
        // Initialize vector of predicted landmarks
        vector<LandmarkObs> pred_landmark;
    
        for (const Map::single_landmark_s & landmark : map_landmarks.landmark_list)
        {
            // Save the landmark if it is within sensor range
            if (dist(landmark.x_f, landmark.y_f, particles[i].x, particles[i].y) <= sensor_range)
                pred_landmark.push_back(LandmarkObs{landmark.id_i, landmark.x_f, landmark.y_f});
        }
    
        // Associate predicted landmarks with observations
        dataAssociation(pred_landmark, world_obs);
    
        // Initialize the new weight
        double weight = 1.0;
    
        for (const LandmarkObs& cur_obs : world_obs)
        {
            // Map id is 1-based indexing
            Map::single_landmark_s cur_pred = map_landmarks.landmark_list[cur_obs.id - 1];

            // Get the square of the difference between the prediction and the measurement
            const double dx_sqr = pow(cur_obs.x - cur_pred.x_f, 2);
            const double dy_sqr = pow(cur_obs.y - cur_pred.y_f, 2);

            // Calculate the multi-variate Gaussian distribution
            const double mvg = 1 / (M_PI * 2 * std_landmark[0] * std_landmark[1]) *
                               exp(-(dx_sqr/ pow(std_landmark[0], 2) + 
                                     dy_sqr / pow(std_landmark[1], 2)));

            // Accumulate the new weight
            weight *= mvg;
        }

        // Assign the new weight
        particles[i].weight = weight;
        weights[i] = weight;
    }
}

void ParticleFilter::resample()
{
    // Random number generator
    default_random_engine gen;

    // Discrete distribution to draw random integers
    discrete_distribution<int> distribution {weights.begin(), weights.end()};

    // Initialize vector for new particles
    vector<Particle> new_particles(num_particles);

    // Draw new particles based on distribution
    for (int i = 0; i < num_particles; ++i)
        new_particles[i] = particles[distribution(gen)];

    // Update the particles
    particles = new_particles;
}

void ParticleFilter::write(string filename)
{
    ofstream dataFile;
    dataFile.open(filename, ios::app);
    for (int i = 0; i < num_particles; ++i)
        dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";

    dataFile.close();
}
