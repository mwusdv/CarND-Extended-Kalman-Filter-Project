#include <iostream>

#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_laser_in, MatrixXd &R_radar_in, 
                        MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Ht_ = H_.transpose();  // pre-calculation
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::laserUpdate(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd S = H_ * P_ * Ht_ + R_laser_;
  MatrixXd K = P_*Ht_ * S.inverse(); 

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::radarUpdateEKF(const VectorXd &z, const MatrixXd& Hj) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  VectorXd z_pred = polar_coord();
  VectorXd y = z - z_pred;

  float two_pi = M_PI*2;
  while (y(1) > two_pi){
    y(1) -= two_pi;
  }
  while (y(1) < -two_pi) {
    y(1) += two_pi;
  }

  MatrixXd Hjt = Hj.transpose();
  MatrixXd S = Hj * P_ * Hjt + R_radar_;

  MatrixXd K = P_*Hjt * S.inverse(); 
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;
}

// get polar coordinates of current state
VectorXd KalmanFilter::polar_coord() const {
  VectorXd z(3);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  z(0) = sqrt(px*px + py*py);
  z(1) = atan2(py, px);
  z(2) = 0;
  if (z(0) > 1e-6) {
    z(2) = (px*vx + py*vy) / z(0);
  }

  return z;
}