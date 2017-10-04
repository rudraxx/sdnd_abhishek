#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "maneuver.h"

using namespace std;


class GNB
{
    public:
        int feature_size;
        int numClasses;

        Maneuver m_left;
        Maneuver m_keep;
        Maneuver m_right;

        vector < vector<double> > mean_values;
        vector < vector<double> > sigma_values;

        GNB();
        virtual ~GNB();

        vector<string> possible_labels= {"left","keep","right"};

        void train(vector<vector<double> > data, vector<string> labels);

        string predict(vector<double> observation);

        double calculate_pdf(vector<double> observation, vector<double> mu,vector<double> stddev);


};

#endif // CLASSIFIER_H
