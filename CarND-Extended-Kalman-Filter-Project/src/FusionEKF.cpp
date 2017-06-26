#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // This needs to be calculated everytime. Initializing as a matrix with first columns as 1.
  Hj_ << 1, 0, 0, 0,
         1, 0, 0, 0,
         1, 0, 0, 0;

  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;



  // //Modify the F matrix so that the time is integrated
  // kf_.F_(0, 2) = dt;
  // kf_.F_(1, 3) = dt;


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    // ekf_.x_ = VectorXd(4);
    // ekf_.x_ << 1, 1, 1, 1;
    Eigen::VectorXd x_in(4,1);

    Eigen::MatrixXd P_in(4,4);
    Eigen::MatrixXd F_in(4,4);
    Eigen::MatrixXd Q_in(4,4);

    P_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    F_in << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;

    // set the process covariance matrix Q
    Q_in <<  1/4*noise_ax, 0, 1/2*noise_ax, 0,
           0, 1/4*noise_ay, 0, 1/2*noise_ay,
           1/2*noise_ax, 0, 1*noise_ax, 0,
           0, 1/2*noise_ay, 0, 1*noise_ay;

    // std::cout << "in initialization loop" << std::endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /*
      For a row containing radar data, the columns are: 
      sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp,
      x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.
      */
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      // std::cout << "Init via radar" << std::endl;

      float term1 = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      float term2 = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);

      x_in << term1,term2,0,0;      

      // Initialize the ekf
      ekf_.Init(x_in, P_in, F_in, Hj_, R_radar_, Q_in);      

    }

    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /*For a row containing lidar data, the columns are: 
        sensor_type, x_measured, y_measured, timestamp,
        x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.
      */
      /**
      Initialize state.
      */
      // std::cout << "Init via Lidar" << std::endl;
      x_in << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1],0,0;      

      // Initialize the ekf
      ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);      

    }
    previous_timestamp_ = measurement_pack.timestamp_;



    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }



  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // std::cout << "Starting prediction loop" << std::endl;
     
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ <<  1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;

  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;


  // std::cout << "Entering ekf_.predict" << std::endl;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

    // std::cout << "Entering ekf_.UpdateEKF in radar" << std::endl;
    // Radar updates
    // Calculate the Hj i.e jacobian matrix
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    // std::cout << "tools.CalculateJacobian(ekf_.x_) done" << std::endl;

    // Need to set the H and R matrices every time because they are different depending on the type of sensor. 
    ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, Hj_, R_radar_, ekf_.Q_);
    // std::cout << "Done ekf init again " << std::endl;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);      

  } else {

    // std::cout << "Entering ekf_.UpdateEKF in Lidar" << std::endl;
    

    // Laser updates
    ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, H_laser_, R_laser_, ekf_.Q_);
    ekf_.Update(measurement_pack.raw_measurements_);      

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
