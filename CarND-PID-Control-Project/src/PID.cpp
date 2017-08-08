#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  is_initialized = false;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  Kp = 0.0;
  Ki = 0.0;
  Kd = 0.0;

}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // Use the init values to update the class variables Kp,Ki,Kd.
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  is_initialized = true;
}

void PID::UpdateError(double cte) {
  // Prev d_error will be the old cte
  d_error = cte - p_error;
  // p_error will be the current cte
  p_error = cte;
  // i_error will be the summation of all cte
  i_error += cte;
}

double PID::TotalError() {
  double err = -1*(Kp * p_error + Ki * i_error + Kd * d_error);
  // Limit the error to -1 to +1
  if (err>1.0){
    err=1.0;
  }
  if (err<-1.0){
    err = -1.0;
  }
  return err;
}
