#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  // std::cout << "  ekf_.predict done" << std::endl;
    
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  // Use this update equation for Lidar update. 
  VectorXd y_ = z - H_ * x_;
  MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();
  x_ = x_ + K_ * (y_);

  long x_size = x_.size();
  // std::cout << "Size of x is : " << x_size << std::endl;

  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  P_ = (I - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // std::cout << "in KalmanFilter::UpdateEKF" << std::endl;
  
  double term1 = sqrt( x_(0) * x_(0) + x_(1) * x_(1) ) ;

  // std::cout << "done  terms 1 " << std::endl;

  double term2 = atan2(x_(1),x_(0));

  double term3 = ( x_(0) * x_(2) + x_(1) * x_(3) ) / term1;

  // std::cout << "done all terms 1,2,3 " << std::endl;

  Eigen::VectorXd hx_(3,1);
  hx_ << term1,term2,term3;

  // std::cout << "hx_ = " << hx_ << std::endl;
  Eigen::VectorXd y_ = (z - hx_);

  // Ensure that the delta theta is in between [-pi,pi]

  float in = y_(1);
  float aa = 3.14;
  float bb = -3.14;
  float out;
  if (in>aa){
      out = in-6.28;
  }
  else if (in<bb){
      out = in + 6.28;
  }
  else{
      out = in;
  }
  y_(1) = out;

  Eigen::MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K_ = P_ * H_.transpose() * S_.inverse();

  x_ = x_ + K_ * (y_);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  P_ = (I - K_ * H_) * P_;    
}
