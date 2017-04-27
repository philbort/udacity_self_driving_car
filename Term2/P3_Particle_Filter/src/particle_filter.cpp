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
    // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation 
    //   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
    //   for the fact that the map's y-axis actually points downwards.)
    //   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample()
{
    // TODO: Resample particles with replacement with probability proportional to their weight. 
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::write(string filename)
{
    ofstream dataFile;
    dataFile.open(filename, ios::app);
    for (int i = 0; i < num_particles; ++i)
        dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";

    dataFile.close();
}
