#include "kalman_filter.h"
#include "tools.h"
#include <math.h>
#include <iostream>

using std::cout;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */


  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;



}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */


	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;



}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   *
   */

	Tools tools2;
	MatrixXd H_ = tools2.CalculateJacobian(x_);

	VectorXd hx(3);
	float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];

	float rho = sqrt (px*px + py*py);
	float phi = atan2(py, px);
	float rho_dot;
	if(fabs(rho) < 0.0001){
		rho_dot = 0;
	} else {
		rho_dot = (px*vx + py*vy)/rho;
	}
	hx << rho, phi, rho_dot;
	cout << "hx ="<< hx << "\n";

	VectorXd y = z-hx;
	while(y(1) > M_PI){
		y(1) -= M_PI;
	}
	while(y(1) < -M_PI){
		y(1) += M_PI;
	}
	MatrixXd H_t = H_.transpose();
	MatrixXd S = H_ * P_ * H_t + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * H_t;
	MatrixXd K = PHt * Si;

	x_ = x_ + (K*y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;



}
