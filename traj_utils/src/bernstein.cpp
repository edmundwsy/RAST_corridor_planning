/**
 * @file bernstein.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-09-06
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "traj_utils/bernstein.hpp"

namespace planner {

/**
 * @brief
 * p(t) = C^T * A * [t^4, t^3, t^2, t, 1]^T
 * C: (N+1)x3 control points
 * A: 5x5 coefficient matrix
 * T: 5x1 time vector
 * @param t
 * @return Eigen::Vector3d
 */
Eigen::Vector3d BernsteinPiece::getPos(double t) const {
  double          s = (t - t0_) / t_;
  Eigen::VectorXd S;
  S.resize(N_ + 1);
  S(0) = 1;
  for (int i = 1; i <= N_; i++) {
    S(i) = pow(s, i);
  }
  return cpts_.transpose() * A_ * S;
}

Eigen::Vector3d BernsteinPiece::getVel(double t) const {
  double          s = (t - t0_) / t_;
  Eigen::VectorXd S;
  S.resize(N_ + 1);
  S(0) = 0;
  S(1) = 1;
  for (int i = 2; i <= N_; i++) {
    S(i) = i * pow(s, i - 1);
  }
  return cpts_.transpose() * A_ * S / t_;
}

Eigen::Vector3d BernsteinPiece::getAcc(double t) const {
  double          s = (t - t0_) / t_;
  Eigen::VectorXd S;
  S.resize(N_ + 1);
  S(0) = 0;
  S(1) = 0;
  S(2) = 2;
  for (int i = 3; i <= N_; i++) {
    S(i) = i * (i - 1) * pow(s, i - 2);
  }
  return cpts_.transpose() * A_ * S / pow(t_, 2);
}

void BernsteinPiece::calcCoeffMat() {
  A_.resize(N_ + 1, N_ + 1);
  A_.setZero();
  switch (N_) {
    case 1:
      A_(0, 0) = 1;
      A_(1, 0) = -1;
      A_(1, 1) = 1;
      break;
    case 2:
      A_ << 1, -2, 1, 0, 2, -2, 0, 0, 1;
      break;
    case 3:
      A_ <<
          // clang-format off
      1, -3, 3, 1,
      0, 3, -6, 3,
      0, 0, 3, -3,
      0, 0, 0, 1;
      // clang-format on
      break;
    case 4:
      A_ <<
          // clang-format off
      1, -4,   6,  -4,  1,
      0,  4, -12,  12, -4,
      0,  0,   6, -12,  6,
      0,  0,   0,   4, -4,
      0,  0,   0,   0,  1;
      // clang-format on
      break;
  }
}


}  // namespace planner