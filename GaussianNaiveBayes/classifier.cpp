#include <iostream>
#include <sstream>
#include <fstream>

#include <vector>
#include "classifier.h"
#include <algorithm>
#include "maneuver.h"

using namespace std;
GNB::GNB()
{
    feature_size = 4;
    numClasses   = 3;

    // Initialize all mean and stddev values to 0
    // I should have vector of <vector of featuresize > of numClasses size

//    vector<double> temp_vector;
//    temp_vector.assign(feature_size, 0);
//
//    for (std::size_t i=0;i<numClasses;++i){
//        mean_values.push_back(temp_vector);
//        sigma_values.push_back(temp_vector);
//    }

//
//    mean_left.assign(feature_size,0);
//    mean_keep.assign(feature_size,0);
//    mean_right.assign(feature_size,0);
//
//    std_left.assign(feature_size,0);
//    std_keep.assign(feature_size,0);
//    std_right.assign(feature_size,0);

    // Initialize 3 maneuvers

//    Maneuver m_left(feature_size);
//    Maneuver m_keep(feature_size);
//    Maneuver m_right(feature_size);

    //ctor
    m_left.set_featureLength(feature_size);
    m_keep.set_featureLength(feature_size);
    m_right.set_featureLength(feature_size);
}

GNB::~GNB()
{
    //dtor
}


//vector<string> possible_labels = {"left","keep","right"}

void GNB::train(vector<vector<double> > data, vector<string> labels){
	/*
		Trains the classifier with N data points and labels.

		INPUTS
		data - array of N observations
		  - Each observation is a tuple with 4 values: s, d,
		    s_dot and d_dot.
		  - Example : [
			  	[3.5, 0.1, 5.9, -0.02],
			  	[8.0, -0.3, 3.0, 2.2],
			  	...
		  	]

		labels - array of N labels
		  - Each label is one of "left", "keep", or "right".
	*/

    // Check if the input data is correct size
    if (labels.size() != data.size() ){
        std::cout<< "Train: Label and data not of same size";
        return;
    }

    // For all data examples
	for (std::size_t i=0;i<data.size(); ++i){
            vector<double> temp_data = data[i];

            if(labels[i]=="left"){
                m_left.add_observation(temp_data);
            }
            else if (labels[i]=="keep"){
                m_keep.add_observation(temp_data);
            }
            else { //(labels[i]=="right"){
                m_right.add_observation(temp_data);
            }
	}

    // Calculate the mean for the data
    m_left.calculate_mean();
    m_keep.calculate_mean();
    m_right.calculate_mean();

	//
    // Calculate the std deviation
    //

    m_left.calculate_stddev();
    m_keep.calculate_stddev();
    m_right.calculate_stddev();

	std::cout<<std::endl;

	std::cout<< "\nTraining Complete\n"<< std::endl;
}



double GNB::calculate_pdf(vector<double> observation, vector<double> mu,vector<double> stddev){

    double temp_var= 1.0;
    double pdf_gaussian = 1.0;
    // Since this is Naive Bayes, we consider the
    // prob(x1,x2,x3,x4 | class) = p(x1|class) * p(x2|class) * p(x3|lass) * p(x4|class)

    for (int i=0;i<feature_size;++i){

        double x      = observation[i];
        double mean   = mu[i];
        double sigma  = stddev[i];

        temp_var = ( 1 / ( sigma * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( ((x - mean)/sigma), 2.0 ) );
        pdf_gaussian *= temp_var;
    }

    return pdf_gaussian;


}

string GNB::predict(vector<double> observation){
    vector<double> prob_classes;

    for (int i=0;i<numClasses; ++i){
        if(i==0){
            // left
            prob_classes.push_back(calculate_pdf(observation,m_left.mean_values,m_left.sigma_values));
        }
        else if (i==1){
            // keep
            prob_classes.push_back(calculate_pdf(observation,m_keep.mean_values,m_keep.sigma_values));
        }
        else{
            // right
            prob_classes.push_back(calculate_pdf(observation,m_right.mean_values,m_right.sigma_values));
        }
    }

    // Calculate sum of all elements:
    double sum_of_elements;
    std::for_each(prob_classes.begin(),prob_classes.end(), [&] (double n){
                  sum_of_elements += n;
    });

    // Normalize all elements in the vector
    for(unsigned int i=0;i<prob_classes.size();++i){
        prob_classes[i] /= sum_of_elements;
    }

    // Find index of larges element
    int idx = std::distance(prob_classes.begin(),std::max_element(std::begin(prob_classes),std::end(prob_classes)));

    return possible_labels[idx];


}



