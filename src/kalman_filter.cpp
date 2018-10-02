#include <iostream>
#include "kalman_filter.h"
#include "tools.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_Laser_in, MatrixXd &R_Laser_in, 
                        MatrixXd &H_Radar_in, MatrixXd &R_Radar_in, 
                        MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_Laser_ = H_Laser_in;
  H_Radar_ = H_Radar_in;
  R_Laser_ = R_Laser_in;
  R_Radar_ = R_Radar_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
    * Laser
  */
  VectorXd y = z - H_Laser_ * x_;
  MatrixXd Ht = H_Laser_.transpose();
  MatrixXd S = H_Laser_ * P_ * Ht + R_Laser_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_Laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
    * Radar
  */
  cout << "UpdateEKF Begin: " << endl;
  Tools tools;

  cout << "x_: " << x_ << endl;
  MatrixXd Hj = tools.CalculateJacobian(x_);
  cout << "Hj: " << Hj << endl;
  double rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  double phi = atan2(x_(1), x_(0));
  double rho_dot = (fabs(rho) < 0.0001) ? 0 : (x_(0) * x_(2) + x_(1) * x_(3)) / rho;

  VectorXd x_polar(3);
  x_polar << rho, phi, rho_dot;
  cout << "x_polar: " << x_polar << endl;
  // VectorXd y = z - Hj * x_polar;
  VectorXd y = z - Hj * x_;
  cout << "y: " << y << endl;
  MatrixXd Ht = Hj.transpose();
  cout << "Ht: " << Ht << endl;
  cout << "P_: " << P_ << endl;
  cout << "R_: " << R_Radar_ << endl;

  MatrixXd S = Hj * P_ * Ht + R_Radar_;
  cout << "S: " << S << endl;
  MatrixXd Si = S.inverse();
  cout << "Si: " << Si << endl;
  MatrixXd K =  P_ * Ht * Si;
  cout << "K: " << K << endl;

  //new state
  x_ = x_ + (K * y);
  cout << "x_: " << x_ << endl;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;
  cout << "P_: " << P_ << endl;
}
