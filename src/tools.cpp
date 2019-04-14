#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   VectorXd rmse(4);
   rmse << 0,0,0,0;

   // TODO: YOUR CODE HERE
   // check the validity of the following inputs:
   //  * the estimation vector size should not be zero
   //  * the estimation vector size should equal ground truth vector size
   if (estimations.size() != ground_truth.size()) {
         throw "estimation has different size from ground truth";
   }

   // accumulate squared residuals
   for (int i=0; i < estimations.size(); ++i) {
         VectorXd gap = estimations[i] - ground_truth[i];
         gap = gap.array() * gap.array();

         rmse += gap;
   }

   // calculate the mean
   rmse = rmse /estimations.size();

   // calculate the squared root
   rmse = rmse.array().sqrt();

   // return the result
   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   MatrixXd Hj(3,4);
   // recover state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   Hj.setZero();

   // compute the Jacobian matrix
   float pho2 = px*px + py*py;
   float pho = sqrt(pho2);
   float pho3 = pho * pho2;

   if (pho > 1e-6) {
      Hj(0, 0) = px / pho;
      Hj(0, 1) = py / pho;
      Hj(1, 0) = -py / pho2;
      Hj(1, 1) = px / pho2;
      Hj(2, 0) = py * (vx*py - vy*px)/pho3;
      Hj(2, 1) = px * (vy*px - vx*py)/pho3;
      Hj(2, 2) = px / pho;
      Hj(2, 3) = py / pho;
   }

   return Hj;
}
