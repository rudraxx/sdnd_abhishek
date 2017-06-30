#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process Noises:
  // Process noise standard deviation longitudinal acceleration in m/s^2
  // consider min max acc of 0.5g = +- 5 m/s2
  std_a_ = 2.0; // this is being overridden by the input arg.

  // Process noise standard deviation yaw acceleration in rad/s^2
  // consider 2pi rad in t seconds. Using s = ut+0.5*a*t^2, u=0; t=10;
  std_yawdd_ = 0.5;

  // Measurement Noises:
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.
  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
  time_us_ = 0.0;

  n_x_ = 5;
  n_aug_ = 7;
  weights_ = VectorXd(2*n_aug_ + 1);
  lambda_ = 3 - n_aug_;
  nis_value = 0.0;

  //set weights for predicted sigma points. Since the weights dont change,
  // initializing them once over here.
  weights_(0) = lambda_/(lambda_+n_aug_);
  // Use vector operation to set weights
  weights_.tail(2*n_aug_).setConstant(1/(2*(lambda_+n_aug_)));

  // number of measurements
  n_z_lidar_ = 2;
  n_z_radar_ = 3;

  //measurement noise R_lidar_ matrix
  R_lidar_ = MatrixXd::Zero(n_z_lidar_,n_z_lidar_);
  //Calculate the measurement noise R_lidar matrix
  R_lidar_(0,0) = std_laspx_*std_laspx_;
  R_lidar_(1,1) = std_laspy_*std_laspy_;

  //measurement noise R_radar_ matrix
  R_radar_ = MatrixXd::Zero(n_z_radar_,n_z_radar_);
  //Calculate the measurement noise R_radar_ matrix
  R_radar_(0,0) = std_radr_*std_radr_;
  R_radar_(1,1) = std_radphi_*std_radphi_;
  R_radar_(2,2) = std_radrd_*std_radrd_;

  std::cout << "UKF initialization complete" << std::endl;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_){
    // Initialize the state vector to match the incoming measurements
    if(meas_package.sensor_type_==MeasurementPackage::LASER){
      // initialize ukf states using lidar measurements
      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];
      x_(2) = 0.0;
      x_(3) = 0.0;
      x_(4) = 0.0;

      // Reviewer suggestion: protect against random init values,
      // in case measurement was faulty
      if ( fabs(x_(0) <= 0.001) && fabs(x_(1)<= 0.001) ) {
        x_(0) = 1;  // initialize with some reasonable value
        x_(1) = 1;
      }

      // initialize the P matrix. Since we are using laser measurement,
      // the covariance for px, and py will be small, since we are confident
      // aout those readings.

      P_.fill(0.0);
      P_(0,0) = std_laspx_ * std_laspx_;
      P_(1,1) = std_laspy_ * std_laspy_;
      P_(2,2) = 20.0;
      P_(3,3) = 20.0;
      P_(4,4) = 1.0;
      // std::cout << " laser piece done " << std::endl;

    }else{
      // initialize ukf states using radar measurements
      x_(0) = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
      x_(1) = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
      x_(2) = meas_package.raw_measurements_[2] ;
      x_(3) = meas_package.raw_measurements_[1];
      x_(4) = 0.0 ;

      // Reviewer suggestion: protect against random init values,
      // in case measurement was faulty
      if ( fabs(x_(0) <= 0.001) && fabs(x_(1) <= 0.001) ) {
        x_(0) = 1;  // initialize with some reasonable value
        x_(1) = 1;
      }

      P_.fill(0.0);
      P_(0,0) = 10;//std_radr_ * std_radr_;
      P_(1,1) = 10;//std_radr_ * std_radr_;
      P_(2,2) = 10.0;
      P_(3,3) = std_radphi_ * std_radphi_;
      P_(4,4) = 1.0;

    }
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;

  }

  // After the system is initialized, extract delta_time
  float delta_t = (meas_package.timestamp_ - time_us_)/1000000.0;
  time_us_ = meas_package.timestamp_ ;

  // std::cout << " About to start prediction function" << std::endl;

  // Reviewer suggestion. If the delta_t is large, that will cause numerical instability
  // So implement multiple prediction steps in case delta_t is large. Nice suggestion.
  // I wonder though why this will cause instability? Because of the small yaw_dd approximation?
  // We arent linearizing as in EKF, so what will be source of error?
  //  Look back at the CRTV model equations.
  // Answer: We are doing Euler integration.
  // Large time step = larger intergation error.
  while (delta_t>0.2){
    double step = 0.05; // 50 millisec timestep
    UKF::Prediction(step);
    delta_t -=step;
  }

  UKF::Prediction(delta_t);

  if(meas_package.sensor_type_==MeasurementPackage::LASER){
    // laser update
    // std::cout<< "Laser measurement" << std::endl;
    if (use_laser_==true){
      UKF::UpdateLidar(meas_package);
    }
  }else{
    // radar update
    // std::cout<< "radar measurement" << std::endl;
    if (use_radar_==true){
      UKF::UpdateRadar(meas_package);
    }
  }


}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  // Set up the augmented matrix
  // Set up the augmented matrix
  VectorXd x_aug_ = VectorXd(n_aug_);
  x_aug_.head(n_x_) = x_;
  x_aug_(n_x_)   = 0.0;
  x_aug_(n_x_+1) = 0.0;

  // Set up the Q matrix for the process noises
  MatrixXd Q_ = MatrixXd::Zero(n_aug_-n_x_,n_aug_-n_x_);
  //     MatrixXd Q_ = MatrixXd(2,2);
  // Q_.fill(0.0);
  Q_(0,0) = std_a_ * std_a_;
  Q_(1,1) = std_yawdd_ * std_yawdd_;

  // Set up the augmented P_aug matrix
  MatrixXd P_aug_ = MatrixXd(n_aug_,n_aug_);
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(n_x_,n_x_) = P_;
  P_aug_.bottomRightCorner(2,2) = Q_;


  // Calculate the sqrt(P_aug_)
  MatrixXd A_ = P_aug_.llt().matrixL();

  // Create the sigma points container
  MatrixXd Xsig_aug = MatrixXd(n_aug_,2*n_aug_+1);

  // Generate the sigmapoints
  Xsig_aug.col(0) = x_aug_;
  // std::cout << " xsig_aug.col(0) done" << std::endl;
  for (int i=0; i<n_aug_;i++){
    Xsig_aug.col(i+1)        = x_aug_ + sqrt(lambda_ + n_aug_) * A_.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * A_.col(i);
  }
  // std::cout << " predict xsig_pred" << std::endl;
  // Set the size of predicted sigma points container
  Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);

  // Predict the Xsig_pred_ by passing the Xsig_aug points through the CTRV model
  for (int i =0;i<2*n_aug_+1;i++){
    // use names so that it is easier to read
    double px     = Xsig_aug(0,i);
    double py     = Xsig_aug(1,i);
    double v      = Xsig_aug(2,i);
    double psi    = Xsig_aug(3,i);
    double psi_d  = Xsig_aug(4,i);
    double nu_a   = Xsig_aug(5,i);
    double nu_psidd = Xsig_aug(6,i);

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
    v     += 0.0 ;
    psi   += psi_d * delta_t;
    psi_d += 0.0 ;

    // add noise
    px +=  0.5*delta_t*delta_t * cos(psi) * nu_a;
    py +=  0.5*delta_t*delta_t * sin(psi) * nu_a;
    v  += delta_t * nu_a;
    psi += 0.5*delta_t*delta_t* nu_psidd;
    psi_d += delta_t * nu_psidd;

    // update the column of Xsig_pred_
    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px;
    Xsig_pred_(1,i) = py;
    Xsig_pred_(2,i) = v;
    Xsig_pred_(3,i) = psi;
    Xsig_pred_(4,i) = psi_d;
  }

  // Based on the predicted sigma points Xsig_pred, calculate the new mean and
  // covariance

  //Predict state mean
  // Calculate the weighted sum for mean. Use matrix * vector math.

  x_ = Xsig_pred_ * weights_;
//  std::cout << "x_  = " << x_.transpose() << std::endl;

  //predict state covariance matrix
  // Calculate the P matrix
  // #bug: init P_ to 0.0 otherwise the value will keep adding values from all
  // previous timesteps. Causes system to hang coz of large values.
  P_.fill(0.0);
  for(int i=0;i<2*n_aug_+1;i++){
      VectorXd x_diff = Xsig_pred_.col(i) - x_;

      //angle normalization
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
      P_ += weights_(i) * x_diff * x_diff.transpose();
  }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  // // set number of measurements
  // int n_z_ = n_z_lidar_;
  // //create matrix for sigma points in measurement space
  // MatrixXd Zsig_ = MatrixXd(n_z_, 2 * n_aug_ + 1);
  //
  // //mean predicted measurement
  // VectorXd z_pred_ = VectorXd(n_z_);
  //
  // //measurement covariance matrix S
  // MatrixXd S_ = MatrixXd(n_z_,n_z_);
  //
  // //transform sigma points into measurement space
  // for (int i=0;i<2*n_aug_+1;i++){
  //     double px = Xsig_pred_(0,i);
  //     double py = Xsig_pred_(1,i);
  //     // double v = Xsig_pred_(2,i);
  //     // double yaw = Xsig_pred_(3,i);
  //     // double yaw_dd = Xsig_pred_(4,i);
  //
  //     Zsig_(0,i) = px;
  //     Zsig_(1,i) = py;
  // }
  // // calculate the weighted mean for measurements
  // z_pred_ = Zsig_ * weights_;
  // S_.fill(0.0);
  // // Calculate the measurement covariance S
  // for (int i=0;i<2*n_aug_+1;i++){
  //     VectorXd z_diff = Zsig_.col(i) - z_pred_;
  //
  //     //angle normalization
  //     while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  //     while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  //
  //     S_ += weights_(i) * z_diff * z_diff.transpose();
  // }
  //
  // S_ += R_lidar_;
  //
  // // create the measurement vector
  // //create example vector for incoming radar measurement
  // VectorXd z_ = VectorXd(n_z_);
  // z_ <<
  //     meas_package.raw_measurements_[0],   //px in m
  //     meas_package.raw_measurements_[1];   //py in m
  //
  // //create matrix for cross correlation Tc
  // MatrixXd Tc_ = MatrixXd(n_x_, n_z_);
  // Tc_.fill(0.0);
  //
  // for (int i=0;i<2*n_aug_+1;i++){
  //     VectorXd x_diff = Xsig_pred_.col(i) - x_;
  //     VectorXd z_diff = Zsig_.col(i) - z_pred_;
  //
  //     Tc_ += weights_(i) * x_diff * z_diff.transpose();
  // }
  //
  // MatrixXd Kk_ = Tc_ * S_.inverse();
  // // Update mean
  // x_ += Kk_ * (z_ - z_pred_);
  //
  // std::cout << "x_  = " << x_.transpose() << std::endl;
  //
  // // Update covariance matrix
  // P_ -= Kk_ * S_* Kk_.transpose();
  //
  // // Calculate the lidar NIS value
  // nis_value =  (z_ - z_pred_).transpose() * S_.inverse() * (z_ - z_pred_);

  // Reviewer comment: Since this is linear, we can just use the standard KF equations to
  // calculate the update. This might be less computationally intensive
  VectorXd z_  = VectorXd(2);
  z_ << meas_package.raw_measurements_[0],meas_package.raw_measurements_[1];

  MatrixXd H_lidar_ = MatrixXd::Zero(n_z_lidar_,n_x_);
  H_lidar_(0,0) = 1;
  H_lidar_(1,1) = 1;

  VectorXd y_ = z_ - H_lidar_*x_;
  MatrixXd S_ = H_lidar_ * P_ * H_lidar_.transpose() + R_lidar_;
  MatrixXd Kk_ = P_ * H_lidar_.transpose() * S_.inverse();
  x_ = x_ + Kk_ * (y_);

  MatrixXd I = MatrixXd::Identity(n_x_,n_x_);
  P_ = (I - Kk_ * H_lidar_) * P_;

  // Calculate the lidar NIS value
  nis_value =  (y_).transpose() * S_.inverse() * (y_);

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  // set number of measurements
  int n_z_ = n_z_radar_;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig_ = MatrixXd(n_z_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred_ = VectorXd(n_z_);

  //measurement covariance matrix S
  MatrixXd S_ = MatrixXd(n_z_,n_z_);


  //transform sigma points into measurement space
  for (int i=0;i<2*n_aug_+1;i++){
      double px = Xsig_pred_(0,i);
      double py = Xsig_pred_(1,i);
      double v = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);
      double yaw_dd = Xsig_pred_(4,i);

      // Reviewer remark: Needs to protect against divide by zero, other
      // numerical issues;
      // If we find the issue, should we throw the full measurement,
      // or calculate the prediction based on the subset of sigma points?
      double term1 = sqrt(px*px + py*py);
      if ( term1<=0.001){
        return ;
      }else{
        Zsig_(0,i) = term1;
        Zsig_(1,i) = atan2(py,px);
        Zsig_(2,i) = (px * v*cos(yaw) + py*v*sin(yaw))/term1;
      }
  }
  // calculate the weighted mean for measurements
  z_pred_ = Zsig_ * weights_;

  // Calculate the measurement covariance S
  S_.fill(0.0);
  for (int i=0;i<2*n_aug_+1;i++){
      VectorXd z_diff = Zsig_.col(i) - z_pred_;

      //angle normalization
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

      S_ += weights_(i) * z_diff * z_diff.transpose();
  }

  S_ += R_radar_;

  // create the measurement vector
  //create example vector for incoming radar measurement
  VectorXd z_ = VectorXd(n_z_);
  z_ <<
      meas_package.raw_measurements_[0],   //rho in m
      meas_package.raw_measurements_[1],   //phi in rad
      meas_package.raw_measurements_[2];   //rho_dot in m/s

  // std::cout<< "z_ = " << z_ << std::endl;
  //create matrix for cross correlation Tc
  MatrixXd Tc_ = MatrixXd(n_x_, n_z_);
  Tc_.fill(0.0);

  for (int i=0;i<2*n_aug_+1;i++){

    // std::cout<< "int i  = " << i << std::endl;

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // std::cout<< "x_diff   = " << x_diff.transpose() << std::endl;
    VectorXd z_diff = Zsig_.col(i) - z_pred_;
    // std::cout<< "z_diff   = " << z_diff.transpose() << std::endl;

    // normalize angles
    while(x_diff(3)>M_PI) x_diff(3) -=2. * M_PI;
    while(x_diff(3)<-M_PI) x_diff(3) +=2. * M_PI;

    while(z_diff(1) >M_PI) z_diff(1) -=2. * M_PI;
    while(z_diff(1)<-M_PI) z_diff(1) +=2. * M_PI;

    Tc_ += weights_(i) * x_diff * z_diff.transpose();
  }
//  std::cout<< "Tc_ = " << Tc_ << std::endl;

  MatrixXd Kk_ = Tc_ * S_.inverse();

  //std::cout<< "Kk_ = " << Kk_ << std::endl;

  // Update mean
  x_ += Kk_ * (z_ - z_pred_);


  // Update covariance matrix
  P_ -= Kk_ * S_* Kk_.transpose();

  // Calculate the radar NIS value
  nis_value =  (z_ - z_pred_).transpose() * S_.inverse() * (z_ - z_pred_);


}
