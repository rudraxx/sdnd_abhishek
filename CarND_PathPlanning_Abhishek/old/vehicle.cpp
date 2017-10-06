
#include "vehicle.h"

Vehicle::Vehicle(double s, double d, double v_in_mps,double dt){
	pos_s = s;
	pos_d = d;
	vel = v_in_mps;
	delta_t = dt;
	last_s = pos_s;
	last_d = pos_d;
}

Vehicle::Vehicle(){
	// do nothing
}

void Vehicle::move(){
	pos_s += vel*delta_t + 0.5* accel*delta_t*delta_t;
	vel += accel* delta_t;
}

void Vehicle::setVars(double s, double d, double v_in_mps,double dt){

	pos_s = s;
	pos_d = d;
	vel = v_in_mps;
	delta_t = dt;
	last_s = pos_s;
	last_d = pos_d;
}

double Vehicle::get_trajectory(double pos_current,double multiplier){
	double pos_new = pos_current + multiplier*vel*delta_t + 0.5* accel*delta_t*delta_t;
	vel += accel* delta_t;
	return pos_new;
}
