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
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;//from Section 12 lesson 5
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd H;
  MatrixXd x;
  MatrixXd P;
  MatrixXd R;
  MatrixXd I;
  VectorXd y = z - H * x;
	MatrixXd Ht = H.transpose();
	MatrixXd S = H * P * Ht + R;
	MatrixXd Si = S.inverse();
	MatrixXd K =  P * Ht * Si;

  //new state
  x = x + (K * y);
  P = (I - K * H) * P; // from Sextion 7 lesson 5
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // EKF requires special calculation of z prediction
  //Section 14 Lesson 5
  //cout << "EKF: " << endl;
  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 1, 1, 1, 1;
  float x = ekf_.x_(0);
  float y = ekf_.x_(1);
  float vx = ekf_.x_(2);
  float vy = ekf_.x_(3);

  float rho = sqrt(x*x+y*y);
  float theta = atan2(x,y);
  float ro_dot = (x*vx+y*vy)/rho;
  VectorXd z_pred = VectorXd(3);
  z_pred << rho.theta.ro_dot;
  VectorXd y = z - z.pred; //this time y is z-z_pred vs
                            // in case of laser y = z - H * x

  //from section 7 in lesson 5
  MatrixXd Ht = H.transpose();
	MatrixXd S = H * P * Ht + R;
	MatrixXd Si = S.inverse();
	MatrixXd K =  P * Ht * Si;

  //new state
  x = x + (K * y);
  P = (I - K * H) * P; // from Sextion 7 lesson 5
}
