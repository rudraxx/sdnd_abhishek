#include <iostream>
#include <sstream>
#include <fstream>

#include <vector>
#include "classifier.h"
#include <algorithm>

using namespace std;
GNB::GNB()
{
    feature_size = 4;
    numClasses   = 3;

    // Initialize all mean and stddev values to 0
    // I should have vector of <vector of featuresize > of numClasses size
    vector<double> temp_vector;
    temp_vector.assign(feature_size, 0);

    for (std::size_t i=0;i<numClasses;++i){
        mean_values.push_back(temp_vector);
        sigma_values.push_back(temp_vector);
    }

//
//    mean_left.assign(feature_size,0);
//    mean_keep.assign(feature_size,0);
//    mean_right.assign(feature_size,0);
//
//    std_left.assign(feature_size,0);
//    std_keep.assign(feature_size,0);
//    std_right.assign(feature_size,0);

    //ctor
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
	int numData = data.size();
	// Create 4 new vectors to store individual class data
	vector<vector<double> > vector_left;
	vector<vector<double> > vector_keep;
	vector<vector<double> > vector_right;

    // Check if the input data is correct size
    if (labels.size() != data.size() ){
        std::cout<< "Train: Label and data not of same size";
        return;
    }

    // For all data examples
	for (unsigned int i=0;i<data.size(); ++i){
            vector<double> temp_data = data[i];

            if(labels[i]=="left"){
                vector_left.push_back(temp_data);
                for (int j=0;j<feature_size; ++j){
                        mean_left[j] += temp_data[j];
                }
            }
            else if (labels[i]=="keep"){
                vector_keep.push_back(temp_data);
                for (int j=0;j<feature_size; ++j){
                        mean_keep[j] += temp_data[j];
                }
            }
            else { //(labels[i]=="right"){
                vector_right.push_back(temp_data);
                for (int j=0;j<feature_size; ++j){
                        mean_right[j] += temp_data[j];
                }
            }
	}

	// Take mean of all the data
    for (std::size_t j=0;j<feature_size; ++j){
        mean_left[j]  = mean_left[j]/numData;
        mean_keep[j]  = mean_keep[j]/numData;
        mean_right[j] = mean_right[j]/numData;
    }

	// printing data for only mean_left
    for (unsigned int i=0;i<mean_left.size(); ++i){
        std::cout<< " Mean_left["<<i<<"]:  "<<mean_left[i]<< "  ";
	}

	std::cout<<std::endl;

	//
    // Calculate the std deviation
    //

    // Left data points
    // Add the all the mean of square deviations
    for (unsigned int i=0;i<vector_left.size(); ++i){
        vector<double> new_vec = vector_left[i];

        for (unsigned int j=0;j<feature_size; ++j){
            std_left[j] +=  pow( (new_vec[j] - mean_left[j]),2) / numData;
        }
    }
    // Calculate the sqrt of the msd
    for (unsigned int j=0;j<feature_size; ++j){
        std_left[j] =  sqrt(std_left[j]);
    }


    // Keep data points
    // Add the all the mean of square deviations
    for (unsigned int i=0;i<vector_keep.size(); ++i){
        vector<double> new_vec = vector_keep[i];
        for (int j=0;j<feature_size; ++j){
            std_keep[j] +=  pow( (new_vec[j] - mean_keep[j]),2) / numData;
        }
    }
    // Calculate the sqrt of the msd
    for (unsigned int j=0;j<feature_size; ++j){
        std_keep[j] =  sqrt(std_keep[j]);
    }

    // Right data points
    // Add the all the mean of square deviations
    for (unsigned int i=0;i<vector_right.size(); ++i){
        vector<double> new_vec = vector_right[i];
        for (int j=0;j<feature_size; ++j){
            std_right[j] +=  pow( (new_vec[j] - mean_right[j]),2) / numData;
        }
    }
    // Calculate the sqrt of the msd
    for (unsigned int j=0;j<feature_size; ++j){
        std_right[j] =  sqrt(std_right[j]);
    }

	// printing data for only mean_left
    for (unsigned int j=0;j<feature_size; ++j){
        std::cout<< " std_left["<<j<<"]:  "<<std_left[j]<< "  ";
	}

	std::cout<<std::endl;

	std::cout<< "\nTraining Complete\n"<< std::endl;
}

void GNB::test(){
	vector<double> asd(4);
	asd.assign(4,10);
	// Print current values
	for (int i=0;i<asd.size(); ++i){
        cout<< asd[i]<< endl;
	}

	vector<double> pp;
	pp.assign(4,5);

	for (int i=0;i<asd.size(); ++i){
            asd[i] += pp[i];
	}

	for (int i=0;i<asd.size(); ++i){
        cout<< asd[i]<< endl;
	}



}

double GNB::calculate_pdf(vector<double> observation, vector<double> mu,vector<double> stddev){

    double temp_var= 1.0;
    double pdf_gaussian = 1.0;
    // Since this is Naive Bayes, we consider the
    // prob(x1,x2,x3,x4 | class) = p(x1|class) * p(x2|class) * p(x3|lass) * p(x4|class)

    for (unsigned int i=0;i<feature_size;++i){

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

    for (unsigned int i=0;i<numClasses; ++i){
        if(i==0){
            // left
            prob_classes.push_back(calculate_pdf(observation,mean_left,std_left));
        }
        else if (i==1){
            // keep
            prob_classes.push_back(calculate_pdf(observation,mean_keep,std_keep));
        }
        else{
            // right
            prob_classes.push_back(calculate_pdf(observation,mean_right,std_right));
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



