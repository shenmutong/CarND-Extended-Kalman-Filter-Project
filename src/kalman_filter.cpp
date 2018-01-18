#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
    * predict the state
  */
  x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  MatrixXd y = MatrixXd(2,1);
  MatrixXd s = MatrixXd(1,1);
  MatrixXd k = MatrixXd(1,1);
  y = (z - H_ *x_);
  s = H_ * P_ * H_.transpose() +R_;
  k = P_ * H_.transpose() * s.inverse();
  // new state
  x_ = x_+ (k *y);
  MatrixXd I_ = MatrixXd::Identity(x_.size(),x_.size());
  P_ = (I_ -k*H_) * P_;
  //std::cout << "x=" << std::endl <<  x << std::endl;
  //std::cout << "P=" << std::endl <<  P << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
