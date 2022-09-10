/**
 * @file bezier_optimizer.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-09-08
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "bernstein/bezier_optimizer.hpp"

namespace traj_opt {

void BezierOpt::setConstraints(const std::vector<PolyhedronH>& constraints) {
  constraints_ = constraints;
  assert(static_cast<int>(constraints_.size()) == M_);
}

void BezierOpt::setTimeAllocation(const std::vector<double>& time_allocation) {
  t_ = time_allocation;
  M_ = t_.size();
}

void BezierOpt::setup(const Eigen::Matrix3d&          start,
                      const Eigen::Matrix3d&          end,
                      const std::vector<double>&      time_allocation,
                      const std::vector<PolyhedronH>& constraints) {
  setTimeAllocation(time_allocation);
  setConstraints(constraints);
  init_ = start;
  end_  = end;

  // x_.resize(DIM * (4 * M_ + 1), 1);
  int DM = DIM * M_ * (N_ + 1);
  // Q_.resize(DM, DM);
  // Q_.setZero();
  A_.resize(10, DM);
  A_.setZero();
  // q_.resize(DM);
  // q_.setZero();
  b_.resize(10);
  b_.setZero();
  x_.resize(DM);
  x_.setZero();
  calcMinJerkCost();
}

void BezierOpt::calcCtrlPtsCvtMat() {
  p2v_.resize(DIM * N_, DIM * (N_ + 1));
  v2a_.resize(DIM * (N_ - 1), DIM * N_);
  a2j_.resize(DIM * (N_ - 2), DIM * (N_ - 1));
  p2v_.setZero();
  v2a_.setZero();
  a2j_.setZero();
  for (int i = 0; i < N_; i++) {
    p2v_.block(i * DIM, i * DIM, DIM, DIM)       = -N_ * Eigen::MatrixXd::Identity(DIM, DIM);
    p2v_.block(i * DIM, (i + 1) * DIM, DIM, DIM) = N_ * Eigen::MatrixXd::Identity(DIM, DIM);
  }
  for (int i = 0; i < N_ - 1; i++) {
    v2a_.block(i * DIM, i * DIM, DIM, DIM)       = -(N_ - 1) * Eigen::MatrixXd::Identity(DIM, DIM);
    v2a_.block(i * DIM, (i + 1) * DIM, DIM, DIM) = (N_ - 1) * Eigen::MatrixXd::Identity(DIM, DIM);
  }
  for (int i = 0; i < N_ - 2; i++) {
    a2j_.block(i * DIM, i * DIM, DIM, DIM)       = -(N_ - 2) * Eigen::MatrixXd::Identity(DIM, DIM);
    a2j_.block(i * DIM, (i + 1) * DIM, DIM, DIM) = (N_ - 2) * Eigen::MatrixXd::Identity(DIM, DIM);
  }
}

/**
 * @brief cost = x'Qx = x'P'QPx
 * x are control points
 * P is the control points to jerk conversion matrix
 *
 */
void BezierOpt::calcMinJerkCost() {
  int DM = DIM * M_ * (N_ + 1);
  Q_.resize(DM, DM);
  Q_.setZero();
  q_.resize(DM);
  q_.setZero();
  Eigen::Matrix<double, DIM, DIM> I = Eigen::MatrixXd::Identity(DIM, DIM);

  Eigen::MatrixXd p2j = a2j_ * v2a_ * p2v_;
  Eigen::MatrixXd P(DIM * (N_ - 2), DIM * (N_ - 2));
  // for (int i = 0; i < N_ - 2; i++) {
  //   P.block<DIM, DIM>(i * DIM, i * DIM) = I / 3;
  //   P.block<DIM, DIM>(i * DIM, (N_ - 3 - i) * DIM) = I / 6;
  // }
  P <<I/3, I/6, I/6, I/3;
  Eigen::MatrixXd QM = p2j.transpose() * P * p2j;
  for (int i = 0; i < M_; i++) {
    Q_.block(i * DIM * (N_ + 1), i * DIM * (N_ + 1), DIM * (N_ + 1), DIM * (N_ + 1)) = QM;
  }
}

void BezierOpt::addContinuityConstraints() {}
void BezierOpt::addDynamicalConstraints() {}
void BezierOpt::addSafetyConstraints() {}

bool BezierOpt::optimize() {
  IOSQP                       solver;
  Eigen::SparseMatrix<double> Q = Q_.sparseView();
  Eigen::SparseMatrix<double> A = A_.sparseView();

  Eigen::VectorXd lb = Eigen::VectorXd::Constant(x_.size(), -OSQP_INFTY);

  c_int flag = solver.setMats(Q, q_, A, lb, b_, 1e-3, 1e-3);

  if (!flag) {
    return false;
  } else {
    solver.solve();
    c_int status = solver.getStatus();
    x_           = solver.getPrimalSol();
    if (status == 1 || status == 2) {
      return true;
    } else {
      return false;
    }
  }
}

}  // namespace traj_opt