#include "kalman_filter.h"
using namespace std;
#include <iostream>
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
  VectorXd y = (z - H_ *x_);
  CommonUpdate(y);
  //MatrixXd s = H_ * P_ * H_.transpose() +R_;
  //MatrixXd k = P_ * H_.transpose() * s.inverse();
  // new state
  //x_ = x_+ (k *y);
  //MatrixXd I_ = MatrixXd::Identity(x_.size(),x_.size());
  //P_ = (I_ -k*H_) * P_;
  //std::cout << "x=" << std::endl <<  x << std::endl;
  //std::cout << "P=" << std::endl <<  P << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd h = VectorXd(3);

  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float pxy_sqrt = sqrtf( px*px+py*py);
  float rho =  pxy_sqrt;
  float phi = 0;
  float rho_dot = 0;
  //phi = atan(py/fabs( px) ) ;
  cout<<"vy  vx" << vx << vx <<endl;
  cout << "z" << z <<endl;
  //cout<<"phi =" <<phi <<endl;


  
  //if(fabs(px) == 0){
  //if( py >0){
  //   phi = M_PI/2;
  // }else if(py <0){
  //  phi = -M_PI / 2 ;
  // }else{
  //   phi =0;
  // }
    //}else{

    //if(px>0 && py >0){
    //   phi = atan2(py,px);
    //}else if(px <0 && py >0){
    //  phi = M_PI -atan2(py, -px) ;
    //}else if (px <0 && py <0){
    //  phi = -(M_PI -atan2(-py , -px));
    //}else{
    //  phi = atan2(py , px);
    //}
    //}

  phi = atan2(py,px);
  /*
  while(phi > M_PI){
    phi -= 2.* M_PI;
  }
  while(phi < -M_PI){
    phi += 2.*M_PI;
    }*/

  if(fabs(pxy_sqrt)  != 0){
    rho_dot = (px * vx + py * vy)/pxy_sqrt;
  }else{
    cout << "pxy_sqrt == 0" <<endl;
    rho_dot = z[2];
  }
  h<< rho,phi,rho_dot;
  //std::cout << "h =  " <<h<<endl;
  //std::cout << "z =  " <<z<<endl;
      
  VectorXd y = z - h;
  NormalizeAngle(y[1]);
  CommonUpdate(y);
  std::cout << "y_ = " << y << endl;
  //MatrixXd s = H_ * P_ * H_.transpose() +R_;
  //MatrixXd k = P_ * H_.transpose() * s.inverse();
  // new state
  //x_ = x_+ (k *y);
  //cout<<"x="<< x_ <<endl;
  // MatrixXd I_ = MatrixXd::Identity(x_.size(),x_.size());
  ///P_ -= K * H_ * P_;
  //P_ = (I_ -k*H_) * P_;

}

void KalmanFilter::NormalizeAngle(double & phi)
{
  phi = atan2(sin(phi),cos(phi));
}

void KalmanFilter::CommonUpdate(const VectorXd& y){
  
  
  const MatrixXd PHt = P_ * H_.transpose();
  const MatrixXd S = H_ * PHt + R_;
  const MatrixXd K = PHt * S.inverse();

  x_ += K * y;
  P_ -= K * H_ * P_;
}

