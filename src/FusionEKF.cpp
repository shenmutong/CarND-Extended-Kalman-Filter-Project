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
  H_laser_ <<  1, 0, 0, 0,
               0, 1, 0, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack){
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
    //cout << "EKF: " << endl;
    //    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;

    MatrixXd P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

    //transition matrix F_
    MatrixXd F_ = MatrixXd(4,4);
    F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;
    MatrixXd Q_ = MatrixXd(4,4);
    // cout<< "Q_ :" << Q_ << endl;
    VectorXd x = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];

      float px = rho * cosf(phi);
      float py = rho * sinf(phi);
      //float vx = rho_dot * cosf(phi);
      // float vy = rho_dot * sinf(phi);
      //      cout << "rho" << rho << "phi" << phi <<"rho_dot"<<rho_dot<<endl;
      x <<  px,py,1,1;
      // Hj_ = tools.CalculateJacobian(x);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x<<  measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],1,1;
    }
    ekf_.Init(x,
              P_,
              F_,
              this->H_laser_,
              this->R_radar_,
              Q_);


    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
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
  //compute the time elapsed between the current and previous measurements
  
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;
  //define noise
  float noise_ax =18;
  float noise_ay =18;
  //Modify the F matrix so that the time is integrated
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
	//Set the process covariance matrix Q
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	//float noise_ax_2 = noise_ax * noise_ax;
	//float noise_ay_2 = noise_ay * noise_ay;  
	ekf_.Q_<< dt_4 / 4 * noise_ax,0,dt_3 / 2*noise_ax,0,
    0, dt_4 / 4*noise_ay,0,dt_3 / 2 *noise_ay,
    dt_3 / 2 * noise_ax, 0,dt_2 * noise_ax,0,
    0,dt_3 / 2 *noise_ay, 0, dt_2 * noise_ay;

  ekf_.Predict();
  //cout << "Predict x_ = " << ekf_.x_ << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    
    // Radar updates
    
    ekf_.R_ = R_radar_;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    cout<< "RADAR :the raw_measurements is :" << measurement_pack.raw_measurements_[2] << endl;
  } else {
    // Laser updates

    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

    // cout<< "LASER :the raw_measurements is :" << measurement_pack.raw_measurements_ << endl;
  }

  // print the output
  //  float rho = measurement_pack.raw_measurements_[0];
  //  float phi = measurement_pack.raw_measurements_[1];
  // float rho_dot = measurement_pack.raw_measurements_[2];
  //  float px = rho * sin(phi);
  // float py = rho*cos(phi);
  // float vx = rho_dot*sin(phi);
  //float vy = rho_dot *cos(phi);
  // cout << "rho" << rho << "phi" << phi <<"rho_dot"<<rho_dot<<endl;
  static float timestamp = 0;
  timestamp += dt;
  //cout << "timeStamp is " << timestamp<< endl;
  // cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}

