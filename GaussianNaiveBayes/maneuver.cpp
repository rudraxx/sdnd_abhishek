#include "maneuver.h"
#include "math.h"

//Maneuver::Maneuver(const int features)
//{
//
//    numFeatures = features;
//    //ctor
//}

Maneuver::Maneuver()
{

    //ctor
}

Maneuver::~Maneuver()
{
    //dtor
}

void Maneuver::set_featureLength(const int features){
    numFeatures = features;

    // Initialize the mean and sigma vectors here
    mean_values.assign(numFeatures ,0);
    sigma_values.assign(numFeatures ,0);


}


void Maneuver::add_observation(const std::vector<double> observation){
    // the observation vector is of the form:
    // s,d,s_dot,d_dot
    // Instead of using d as a feature, lets use d/lane_width as a feature.
    // Create a new vector, since can't modify the input. Using const identifier
    vector <double> tempObservation = observation;
    tempObservation[1] /= lane_width;

    vector_observations.push_back(tempObservation);


}

void Maneuver::calculate_mean(){

    std::size_t numObservations = vector_observations.size();
    for (std::size_t i=0;i<numObservations; ++i){
        vector <double> temp_data =  vector_observations[i];

        for (int j=0;j<numFeatures; ++j){
            mean_values[j] += temp_data[j];
        }
    }

    // Do the /numObservations outside the for loop. Otherwise we will be doing divide operation in each loop.
    for (int j=0;j<numFeatures; ++j){
        mean_values[j] /= numObservations;
    }

}

void Maneuver::calculate_stddev(){

    std::size_t numObservations = vector_observations.size();

    for (unsigned int i=0;i<numObservations; ++i){
        vector<double> new_vec = vector_observations[i];

        for (int j=0;j<numFeatures; ++j){

            sigma_values[j] += pow( (new_vec[j] - mean_values[j]),2) / numObservations;
        }
    }
    // Calculate the sqrt of the msd
    for (int j=0;j<numFeatures; ++j){
        sigma_values[j] =  sqrt(sigma_values[j]);
    }

}

