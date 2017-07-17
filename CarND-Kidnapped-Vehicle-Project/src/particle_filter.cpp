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

	std::cout<< "GPS position: x: " << x<<"   y: "<< y<< std::endl;
	std::cout<< "std_x: " << std[0]<< std::endl;

	num_particles=200;
	//error: normal_distribution takes (mean,stddev) as inputs! NOT current_position and stddev! Arghh
	// either use mean as 0, and add that to the x, or use x as mean, and assign dist_x(generator) value
	// to particle.x
	// std::default_random_engine generator;
	std::normal_distribution<double> dist_x(x,std[0]);
	std::normal_distribution<double> dist_y(y,std[1]);
	std::normal_distribution<double> dist_theta(theta,std[2]);

	for (int i=0;i<num_particles;i++){
		Particle temp_particle;
		temp_particle.id = i;
		temp_particle.x = dist_x(generator);
		temp_particle.y = dist_y(generator);
		temp_particle.theta = dist_theta(generator);
		temp_particle.weight = 1.0;
		particles.push_back(temp_particle);

	}
	is_initialized = true;
	// std::cout<< "Init 	: 	";
	// for (int ii=0;ii<particles.size();ii++){
	// 	std::cout<< "	,weight="<< particles[ii].weight;
	// }
	// std::cout<<""<<std::endl;
	// std::cout<< "Curr_ptcle x: " <<particles[0].x <<", y : 	"<<particles[0].y<<std::endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	// std::cout<<"in Prediction step"<<std::endl;

	// std::default_random_engine generator;
	double v      = velocity ;
	double psi_d  = yaw_rate;

	for (int i=0;i<num_particles;i++){
		// use names so that it is easier to read
		double px     = particles[i].x;
		double py     = particles[i].y;
		double psi    = particles[i].theta;
		double current_weight  = particles[i].weight;


		// Check if psydot==0
		if (fabs(psi_d)<0.001){
		    // update states excluding noise
		    px += v * cos(psi) * delta_t;
		    py += v * sin(psi) * delta_t;

		}else{
		    double term1 = v/psi_d;
		    double term2 = psi + delta_t*psi_d;

		    px += term1 * (sin(term2) - sin(psi));
		    py += term1 * (-cos(term2) + cos(psi));

		}
		psi   += psi_d * delta_t;

		// particles[i].x = px;
		// particles[i].y = py;
		// particles[i].theta = psi; // question: Do I need to do psi%(2pi)?

		// Add gaussian noise to the px,py and psi. Use std_pos[] for error.
		// Consider px,py,psi as the mean, and generate noise based on std_pos
		std::normal_distribution<double> dist_px(px,std_pos[0]);
		std::normal_distribution<double> dist_py(py,std_pos[1]);
		std::normal_distribution<double> dist_psi(psi,std_pos[2]);

		particles[i].x = dist_px(generator);
		particles[i].y = dist_py(generator);
		particles[i].theta = dist_psi(generator);
	}
	// std::cout<< "Predict 	: 	";
	// for (int ii=0;ii<num_particles;ii++){
	// 	std::cout<< "	,weight="<< particles[ii].weight;
	// }
	// std::cout<<""<<std::endl;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations) {
	// std::cout<<"in data association "<<std::endl;
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

	// int min_pred_obs = std::min(observations.size(),predicted.size());
	// For each transformed observation, find the closest prediction/ map_landmark
	for (int i=0;i<observations.size();i++){
		double min_distance = 1000.0;
		// double cutoff_distance = 2.0;
		// Loop through all observations
		LandmarkObs curr_obs = observations[i];

		for (int j=0;j<predicted.size();j++){
			LandmarkObs curr_pred = predicted[j];

			double distance = dist(curr_pred.x,curr_pred.y,curr_obs.x,curr_obs.y);
			if (distance<min_distance){
				// imp: Do assignments on the original array of struct,
				//  not the instance curr_obs
				observations[i].id = curr_pred.id;
				min_distance = distance;
				// std::cout<<"min dist: "<< min_distance<< std::endl;

			}
		}
		// std::cout<<"pred complete"<<std::endl;

	}
	// std::cout<< "Association : 	";
	// for (int ii=0;ii<observations.size();ii++){
	// 	std::cout<< "	,id ="<< observations[ii].id;
	// }
	// std::cout<<""<<std::endl;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {
			// std::cout<<"in updateweights"<<std::endl;
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


	// For each particle
	for (int i=0;i<num_particles;i++){
		std::vector <LandmarkObs> obs_transformed_list;
		LandmarkObs trans_obs;

		Particle current_particle = particles[i];
		// std::cout<< "first Curr_ptcle x: " <<current_particle.x <<", y : 	"<<current_particle.y<<std::endl;

		// Step 1: Transform the observations from body to world coordinates.
		// For each observation convert the observation to world coordinates
		double ct = cos(current_particle.theta);
		double st = sin(current_particle.theta);

		for(int j=0;j<observations.size();j++){

			trans_obs.x =  current_particle.x + ct * observations[j].x  - st * observations[j].y;
			trans_obs.y =  current_particle.y + st * observations[j].x + ct * observations[j].y;

			// create vector of transformed observations
			obs_transformed_list.push_back(trans_obs);
		}
		// std::cout<< "Number of obs : 	"<<obs_transformed_list.size()<<std::endl;

		// std::cout<< " trans obs : 	"<<std::endl;
		// for (int ii=0;ii<obs_transformed_list.size();ii++){
		// 	std::cout<< "x ="<< obs_transformed_list[ii].x<< "	,y ="<< obs_transformed_list[ii].y<< std::endl;
		// }
//		std::cout<<""<<std::endl;


		// Now that we have the observations transformed into world coordinates,
		// find out which landmark they are closest to.

		// Step 2: For this, see which landmarks are within sensor range for this particle.
		// Remember - all this is in world coordinates now.
		std::vector<LandmarkObs> predicted_list;

		// std::cout<< "Curr_ptcle x: " <<current_particle.x <<", y : 	"<<current_particle.y<<std::endl;
		for (int k=0;k<map_landmarks.landmark_list.size();k++){
			Map::single_landmark_s current_landmark = map_landmarks.landmark_list[k];
			double x_dist = current_particle.x - current_landmark.x_f;
			double y_dist = current_particle.y - current_landmark.y_f;

			double distance = sqrt(x_dist*x_dist + y_dist*y_dist);
			if (distance<sensor_range){
				// std::cout<< "lmarkid : " << k+1<<", distance : 	"<<distance<<std::endl;
				// add this potential predicted observation to the predicted_list
				LandmarkObs pred_obs;
				pred_obs.x = current_landmark.x_f;
				pred_obs.y = current_landmark.y_f;
				pred_obs.id = current_landmark.id_i;

				predicted_list.push_back(pred_obs);
			}
		}
		// std::cout<< "Number of pred : 	"<<predicted_list.size()<<std::endl;

		// std::cout<< " pred obs : 	"<<std::endl;
		// for (int ii=0;ii<predicted_list.size();ii++){
		// 	std::cout<< "	,x ="<< predicted_list[ii].x<< "	,y ="<< predicted_list[ii].y<< std::endl;
		// }

		// std::cout<< "pred creation : 	";
		// for (int ii=0;ii<predicted_list.size();ii++){
		// 	std::cout<< "	,lmarkid ="<< predicted_list[ii].id;
		// }
		// std::cout<<""<<std::endl;

		// Step 3: Now associate the predicted_list to observations.
		ParticleFilter::dataAssociation(predicted_list, obs_transformed_list);

		// Step 4: Calculate propability of each measurement and find the total probability of given particle.
		double probability = 1.0;
		// Calculate the denominator ( sqrt(2pi) * sig_x * sig_y)
		double denominator = sqrt(2*M_PI) * std_landmark[0] * std_landmark[1];

		// Get the correct values of mu_x and mu_y based on the associations.

		for(int l=0;l<obs_transformed_list.size();l++){
			LandmarkObs obs = obs_transformed_list[l];
			double mu_x;
			double mu_y;

			for (int m = 0; m < predicted_list.size(); m++){
				if (predicted_list[m].id == obs.id){
					mu_x = predicted_list[m].x;
					mu_y = predicted_list[m].y;
				}
			}
			double x_term = (obs.x - mu_x)/std_landmark[0];
			double y_term = (obs.y - mu_y)/std_landmark[1];
			// std::cout<< "x_term : 	"<<x_term<<" y_term : "<< y_term<<std::endl;

			// Imp - I think we need to make sure to update weight on the particles[i], and not
			// current_particle. Although they are the same, we won't be updating the particles struct
			// if we operate on current_particle.
			// error2: was directly assigning to particles[i] =*=.
			// this will cause error, since it wil keep updating the preious weight, so will drop to 0 very fast.

			probability *= exp( -0.5* (x_term*x_term + y_term*y_term)) / denominator;
			// std::cout<< "probability : 	"<<probability<<std::endl;
		}
		particles[i].weight = probability;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::vector<double> weights_array;
	std::vector<Particle> particles3;


	// Create array of weights
	for (int i=0;i<particles.size();i++){
		weights_array.push_back(particles[i].weight);
	}
	// std::default_random_engine generator;
	// double asd[3] = {11.1,2,3.3};
	std::discrete_distribution <> dist_weights(weights_array.begin(),weights_array.end());//{2.2,3.0,4.0,5.0,1.0}; //(weights_array);

	// For all particles
	for (int i=0;i<particles.size();i++){
		int idx = dist_weights(generator);
		// Set the new particle
		particles3.push_back(particles[idx]);
	}

	// set particles3 as particles
	particles = particles3;
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


// junk
// If a landmark is within the sensor range,
// Transform coordinates, and add that to the predicted observations vector.
// Now that we have the full vector of predictions in world coordinates,
// associate the predictions with necessary world feature.

// // For all landmarks
// for (int j=0;j<map_landmarks.landmark_list.size();j++){
// 	double distance = dist(particles[i].x,particles[i].y,map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f);
// 	if (distance<sensor_range){
// 		// Transform the map_landmark into body coordinates
// 		double ct = cos(particles[i].theta);
// 		double st = sin(particles[i].theta);
// 		// Note that the transform matrix is [c s;-s c],
// 		// instead of [c -s;s c] for body to world
// 		double xx = (map_landmarks.landmark_list[j].x_f - particles[i].x);
// 		double yy = (map_landmarks.landmark_list[j].y_f - particles[i].y);
// 		temp_obs.x =  ct * xx + st * yy;
// 		temp_obs.y =  -st * xx + ct * yy;
// 		temp_obs.id = map_landmarks.landmark_list[j].id_i;
//
// 		predictedObs.push_back(temp_obs);
// 	}
// }

// 	std::random_device rd;
//   std::mt19937 gen(rd());
//   std::discrete_distribution<> d(weights_array.begin(), weights_array.end());
//   for(int n=0; n<num_particles; ++n) {
//       Particle particle_res = particles[d(gen)];
//       particles3.push_back(particle_res);
//   }
// 	particles = particles3;
// }
