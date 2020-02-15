/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 * Modified by: Raymond Andrade
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::default_random_engine;
using std::numeric_limits;

default_random_engine rand_gen; // random num generator

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles
  
  // Noise Distribution For Sensors
  normal_distribution<double> norm_x_init(0, std[0]);
  normal_distribution<double> norm_y_init(0, std[1]);
  normal_distribution<double> norm_theta_init(0, std[2]);
  
  // Create Random Particles
  for (int i = 0; i < num_particles; i++) {
    Particle temp_particle;
    temp_particle.id = i;
    temp_particle.x = x + norm_x_init(rand_gen);
    temp_particle.y = y + norm_y_init(rand_gen);
    temp_particle.theta = theta + norm_theta_init(rand_gen);
    temp_particle.weight = 1.0;
    
    
    particles.push_back(temp_particle); // particles vector from main.cpp
  
  }
  
  is_initialized = true;  // Set init flag in particle_filter.h to true
    
  std::cout << "Initializaed Particle Filter" << std::endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  // Noise Distribution For Sensors
  normal_distribution<double> norm_x(0, std_pos[0]);
  normal_distribution<double> norm_y(0, std_pos[1]);
  normal_distribution<double> norm_theta(0, std_pos[2]);
  
  // Adjust each particle to incorporate velocity and yaw angle
  for (int i = 0; i < num_particles; i++) {
    
    // Check Yaw rate since can't divide by 0 
    if (fabs(yaw_rate) > 0.00001) { 
      particles[i].x += (velocity/yaw_rate) * (sin(particles[i].theta + (yaw_rate*delta_t)) - sin(particles[i].theta));
      particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)));
      particles[i].theta += yaw_rate * delta_t;
    
    } else {
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    }
    
    // Take into account noise of measurment values
    particles[i].x += norm_x(rand_gen);
    particles[i].y += norm_y(rand_gen);
    particles[i].theta += norm_theta(rand_gen);
  }  

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  
  // Loop through all observations 
  for (int i = 0; i < observations.size(); i++) {
    
    LandmarkObs current_obs = observations[i];
    
    // Define the minimum distance
    double min_dist = numeric_limits<double>::max();
    
    
    // Loop through all predictions to comapre to the observation
    for (int j = 0; j < predicted.size(); j++) {
      LandmarkObs current_pred = predicted[j];
      
      // Calculate distance between observed coords, and predicted coordinates
      double current_dist = dist(current_obs.x, current_obs.y, current_pred.x, current_pred.y); // dist() function from helper_functions.h
      
      // If the distance between current predicted is smaller than that of previous predicted landmarks, set it as the associated observation point and update min_dist
      if (current_dist < min_dist){
      	min_dist = current_dist;
        observations[i].id = current_pred.id;
      } // End if statement
      
    } // End looping through predictions
  } // End looping through observations
} // End dataAssociation

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  
   // Loop through each particle
  for (int i = 0; i < num_particles; i++) {

    //// Grab info of each particle
    double part_x = particles[i].x;
    double part_y = particles[i].y;
    double part_theta = particles[i].theta;

    // Create a vector for the landmark predictions
    vector<LandmarkObs> landmark_predictions;
   
    
    
    // Loop through each landmark
    for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {

      // Grab info of each landmark
      float land_x = map_landmarks.landmark_list[j].x_f;
      float land_y = map_landmarks.landmark_list[j].y_f;
      int land_id = map_landmarks.landmark_list[j].id_i;

      // Only consider points which are whithin the sensors range
      // fabs() is the abasolute value functi// on for C++ floating number, which took me a while to figure out. I miss python
      if (fabs(land_x - part_x) <= sensor_range && fabs(land_y - part_y) <= sensor_range) {
      
        // Add prediction to landmark vector
        landmark_predictions.push_back(LandmarkObs{ land_id, land_x, land_y });
        
      }
    }
      
      // Create a vector for the transform// ed landmark prediction in map coordinates
      vector<LandmarkObs> transformed_obs;
      
      // Do Transformation of Coordinates relative to car to Coordinates relative to map 
      //Loop through each observon
      for (int k = 0; k < observations.size(); k++){
       
        // Transform Equations Derived from transm matrix
        float transformed_x = cos(part_theta)*observations[k].x - sin(part_theta)*observations[k].y + part_x;
        float transformed_y = sin(part_theta)*observations[k].x + cos(part_theta)*observations[k].y + part_y;

        transformed_obs.push_back(LandmarkObs{observations[k].id, transformed_x, transformed_y});
      }
      
      // Match predicted measurements with closestansformed measuremnts. 
      dataAssociation(landmark_predictions, transformed_obs);
      
      // Reinitialize weights to prepare  update (weights between 0-1)
      particles[i].weight = 1.0;
      
      // Loop throgh each transformed observed landmark
      for (int l = 0;  l < transformed_obs.size(); l++) {
      
        double pred_x;
        double pred_y;
    
        // Loop through eaprediction for the current observation landmark
        for (int m=0; m < landmark_predictions.size(); m++) {
          if (landmark_predictions[m].id == transformed_obs[l].id) {
          	pred_x = landmark_predictions[m].x; 
            pred_y = landmark_predictions[m].y; 
          }
        }
        
        // Abreviated variables to make the observation weighquation more readable
        double trans_x= transformed_obs[l].x;
        double trans_y = transformed_obs[l].y; 
        double std_x = std_landmark[0];
      	double std_y = std_landmark[1];
        
     	// Multivariate-Gaussian Probability Density equation used to calculate weight
      	double observation_weight = ( 1/(2*M_PI*std_x*std_y)) * exp( -( pow(pred_x-trans_x,2)/(2*pow(std_x, 2)) + (pow(pred_y-trans_y,2)/(2*pow(std_y, 2))) ) );
        

      	// Update theight of the particle
      	particles[i].weight *= observation_weight;
     
        
      } // End of transformed observed landmark loop
    } // End of particle loop
} // End of dateWeights function

void ParticleFilter::resample() {
  /**
   // * TODO: Resample particles with replacement with probabi// lity proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  
  vector<Particle> resampled_part;
  resampled_part.resize(num_particles);
  
  vector<double> part_weights;
  
  // Loop through each particle and grab the weights
  for (int i = 0; i < num_particles; i++) {
  
    part_weights.push_back(particles[i].weight);
  
  }
  
  // Sort out the max weight in the particle weights vector
  // Reference: https://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights.begin(), weights.end());
  
  // Declare Index variable
  int resampled_index;
  
  // Loop through each particle and resample based off discrete distribution
  for (int i = 0; i < num_particles; i++) {
    resampled_index = d(gen);
    resampled_part[i] = particles[resampled_index];
  }
  
  // Assign the new resampled particles to the particles vector
  particles = resampled_part;

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}