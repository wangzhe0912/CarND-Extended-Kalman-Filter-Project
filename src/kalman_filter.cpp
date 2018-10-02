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
  cout << "I will predict" << endl;
  cout << "input: F_: " << F_ << endl;
  cout << "input: x_: " << x_ << endl;
  cout << "input: Q_: " << Q_ << endl;
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  cout << "output: x_: " << x_ << endl;
  cout << "output: P_: " << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
    * Laser
  */
  cout << "Update Begin: " << endl;
  cout << "x_: " << x_ << endl;
  VectorXd y = z - H_Laser_ * x_;
  cout << "H: " << H_Laser_ << endl;
  cout << "y: " << y << endl;
  cout << "R: " << R_Laser_ << endl;
  MatrixXd Ht = H_Laser_.transpose();
  MatrixXd S = H_Laser_ * P_ * Ht + R_Laser_;
  MatrixXd K =  P_ * Ht * S.inverse();
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());


  //new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_Laser_) * P_;
  cout << "Update output: " << endl;
  cout << "output: x_: " << x_ << endl;
  cout << "output: P_: " << P_ << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
    * Radar
  */
  cout << "UpdateEKF Begin: " << endl;
  Tools tools;
  double rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  double phi = atan2(x_(1), x_(0));
  double rho_dot = (fabs(rho) < 0.0001) ? 0 : (x_(0) * x_(2) + x_(1) * x_(3)) / rho;

  VectorXd x_polar(3);
  x_polar << rho, phi, rho_dot;
  cout << "x_polar: " << x_polar << endl;

  MatrixXd Hj = tools.CalculateJacobian(x_);
  cout << "H: " << Hj << endl;
  // VectorXd y = z - Hj * x_polar;
  VectorXd y = z - x_polar;
  cout << "y: " << y << endl;
  cout << "R: " << R_Radar_ << endl;
  MatrixXd Ht = Hj.transpose();
  

  MatrixXd S = Hj * P_ * Ht + R_Radar_;
  MatrixXd K =  P_ * Ht * S.inverse();
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());


  //new state
  x_ = x_ + (K * y);
  P_ = (I - K * Hj) * P_;
  cout << "UpdateEKF output: " << endl;
  cout << "output: x_: " << x_ << endl;
  cout << "output: P_: " << P_ << endl;
}
