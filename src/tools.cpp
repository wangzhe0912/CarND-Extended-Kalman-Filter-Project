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
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  // ... your code here
  
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
      cout << "input not valid" << endl;
      return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
        // ... your code here
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
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
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
  double threshold = 0.1;
  if ((px < threshold && px > -threshold) || (py < threshold && py > -threshold)) {
    Hj << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    return Hj;
  }
  //compute the Jacobian matrix
  else {
    double squared = pow(px, 2) + pow(py, 2);
    // square root of summed squares
    double root = sqrt(squared);
    // squared * root
    double factor = squared * root;

    /** Calculate a Jacobian */
    Hj << px / root, py / root, 0, 0,
          -py / squared, px / squared, 0, 0,
          py * (vx * py - vy * px) / factor, px * (vy * px - vx * py) / factor, px / root, py / root;
    return Hj;
  }
}
