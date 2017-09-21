#ifndef MANEUVER_H
#define MANEUVER_H
#include <vector>

using namespace std;
class Maneuver
{
    public:

        int numFeatures;
        double lane_width = 4.0;
        vector<double> mean_values;
        vector<double> sigma_values;

        vector<vector<double> > vector_observations;

        Maneuver();
//        Maneuver(const int features);

        virtual ~Maneuver();

        void set_featureLength(const int features);

        void add_observation(vector<double> observation);

        void calculate_mean();

        void calculate_stddev();

};

#endif // MANEUVER_H
