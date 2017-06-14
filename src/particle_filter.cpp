/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
  //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  
  // set inital number of particles to 1000
  num_particles = 1000;

  // Random engine to generate psudo random numbers
  default_random_engine gen;

  // Normal (Gaussian) distribution for x
  normal_distribution<double> dist_x(x, std[0]);
  // Normal distributions for y
  normal_distribution<double> dist_y(y, std[1]);
  // Normal distributions for psi
  normal_distribution<double> dist_theta(theta, std[2]);

  // Generate particles
  for (int i = 0; i < num_particles; ++i) {
    double sample_x, sample_y, sample_theta;

    // Create new particle
    Particle particle = {
      .id = i,
      .x = dist_x(gen),
      .y = dist_y(gen),
      .theta = dist_theta(gen),
      .weight = 1.0,
      // .associations.push_back(?),
      // .sense_x.push_back(?),
      // .sense_y.push_back(?),
    };
    
    particles.push_back(particle);
    weights.push_back(1);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  // Random engine to generate psudo random numbers
  default_random_engine gen;
  double new_x, new_y, new_theta;

  for(int i; i < num_particles; i++){
    // Prediction based on yaw_rate
    if(yaw_rate == 0){
      new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
      new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
      new_theta = particles[i].theta;
    }
    else{
      new_x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
      new_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta + yaw_rate * delta_t) - cos(particles[i].theta));
      new_theta = particles[i].theta + yaw_rate * delta_t;
    }

    // Add jittered noise
    normal_distribution<double> dist_x(new_x, std_pos[0]);
    normal_distribution<double> dist_y(new_y, std_pos[1]);
    normal_distribution<double> dist_theta(new_theta, std_pos[2]);

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
  //   implement this method and use it as a helper during the updateWeights phase.
  std::vector<LandmarkObs>::const_iterator obs_iter;
  std::vector<LandmarkObs>::const_iterator pre_iter;
  for(obs_iter = observations.begin();obs_iter!=observations.end();++obs_iter) {
    double distance = 99999999;
    for(pre_iter = predicted.begin();pre_iter!=predicted.end();++pre_iter) {
      if (dist(obs_iter->x,obs_iter->y,pre_iter->x,pre_iter->y) < distance) {
          distance = dist(obs_iter->x,obs_iter->y,pre_iter->x,pre_iter->y);
          obs_iter->id = pre_iter->id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
    std::vector<LandmarkObs> observations, Map map_landmarks) {
  // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
  //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  //   according to the MAP'S coordinate system. You will need to transform between the two systems.
  //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
  //   The following is a good resource for the theory:
  //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //   and the following is a good resource for the actual equation to implement (look at equation 
  //   3.33
  //   http://planning.cs.uiuc.edu/node99.html
  // Normalize

  for(int i; i < num_particles; i++){

    // Transform
    vector<LandmarkObs> trans_observations;
    LandmarkObs obs;
    for(int j = 0; j < observations.size(); j++){
      LandmarkObs trans_obs;
      obs = observations[j];

      // coord transformation from vehicle coord to map coord
      trans_obs.x = cos(particles[i].theta) * obs.x - sin(particles[i].theta) * obs.y + particles[i].x;
      trans_obs.y = sin(particles[i].theta) * obs.x + cos(particles[i].theta) * obs.y + particles[i].y;
      trans_observations.push_back(trans_obs);
    }

    particles[i].weight = 1.0;

    // get the nearest point on map
    vector<LandmarkObs> associated_LandMarks;
    dataAssociation(trans_observations, )

    // update weights
    for(int j = 0; j < trans_observations.size(); j++){

      int id = 0;
      for(int k = 0; k < map_landmarks.size(); k++){
        dist()
      }
      // calc multi-variant gaussian distribution prob
      // 
    }

    if(association != 0){
      double measure_x = trans_observations[j].x;
      double measure_y = trans_observations[j].y;
      double mu_x = map_landmarks.landmark_list[association].x_f;
      double mu_y = map_landmarks.landmark_list[association].y_f;
      long double probability = BivariantGaussian(measure_x, measure_y, mu_x, mu_y, std_landmark[0], std_landmark[1]);
      if(probability > 0){
        particles[i].weight *= probability;
      }
      associations.push_back(association + 1);
      sense_x.push_back(trans_observations[j].x);
      sense_y.push_back(trans_observations[j].y)
    }


  }
}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to their weight. 
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  default_random_engine gen;
  discrete_distribution<int> distribution(weights.begin(), weights.end());

  vector<Particle> resample_particles;

  for(int i = 0; i < num_particles; i++){
    resample_particles.push_back(particles[distribution(gen)]);
  }

  particles = resample_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
  //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  //Clear the previous associations
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;

  return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseX(Particle best)
{
  vector<double> v = best.sense_x;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseY(Particle best)
{
  vector<double> v = best.sense_y;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
