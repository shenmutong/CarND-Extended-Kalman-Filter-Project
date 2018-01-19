#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  /**
    * Calculate the RMSE.
  */

  VectorXd rmse(4);
	rmse << 0,0,0,0;

  // TODO: check estimations
  if(estimations.size()<=0 || estimations.size() != ground_truth.size()){
    return rmse;
  }
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
    // ... your code here
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
	}

	//calculate the mean
  rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian.
  */
  MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float pxy = px*px + py* py;
	float pxy_sqrt = sqrt(pxy); 
  float pxy_3_2 = pxy * pxy_sqrt;

	//check division by zero
  if(fabs(pxy) < 0.0001){
      return Hj;
	}

	//compute the Jacobian matrix
  Hj << px/pxy_sqrt,py/pxy_sqrt,0,0,
    px/pxy, py/pxy,0,0,
    py*(vx*py - vy*px)/pxy_3_2,px*(vy*px - vx*py)/pxy_3_2,px/pxy_sqrt,py/pxy_sqrt;
	return Hj;
}
