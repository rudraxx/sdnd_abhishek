#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <vector> 
using namespace std;

class Vehicle {

public:
	double pos_s;
	double pos_d;
	double vel; //  in m/s
	double accel;

	double delta_t;
	double last_s;
	double last_d;

	vector<double> prev_traj_x;
	vector<double> prev_traj_y;
	vector<double> prev_traj_s;
	vector<double> prev_traj_d;


	bool is_initialized = false;

	Vehicle();

	Vehicle(double s, double d, double v_in_mps,double delta_t);

	void move();

	void setVars(double s, double d, double v_in_mps,double delta_t);
	
	double get_trajectory(double pos_current, double multiplier);

};

#endif