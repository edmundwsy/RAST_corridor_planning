/**
 * @file bezier_optimizer.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-09-08
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _BEZIER_OPTIMIZER_H_
#define _BEZIER_OPTIMIZER_H_

// #define DIM 3

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <memory>
#include <traj_utils/bernstein.hpp>
// #include <sdqp.hpp>
#include <vector>
#include <iosqp.hpp>

namespace traj_opt {

typedef Bernstein::Bezier BezierCurve;
/**
 * @brief H-representation of a polytope
 * Each row is a constraint
 * h0 * x + h1 * y + h2 * z + h3 <= 0
 */
typedef Eigen::MatrixX4d  PolyhedronH;

class BezierOpt {
 public:
  BezierOpt() {
    N_ = ORDER;  // 4
    calcCtrlPtsCvtMat();
  }
  BezierOpt(const int& N) {
    N_ = N;
    calcCtrlPtsCvtMat();
  }
  ~BezierOpt() {}

  /* main API */
  void setConstraints(const std::vector<PolyhedronH>& constraints);
  void setTimeAllocation(const std::vector<double>& time_allocation);

  void addConstraints();
  void addContinuityConstraints();
  void addDynamicalConstraints();
  void addSafetyConstraints();
  void calcCtrlPtsCvtMat();
  void calcMinJerkCost();

  void setup(const Eigen::Matrix3d& start, const Eigen::Matrix3d& end,
             const std::vector<double>&          time_allocation,
             const std::vector<PolyhedronH>& constraints);
  bool optimize();

  /* getters */
  inline Eigen::MatrixXd getPos2VelMat() { return p2v_; }
  inline Eigen::MatrixXd getVel2AccMat() { return v2a_; }
  inline Eigen::MatrixXd getAcc2JerkMat() { return a2j_; }
  inline Eigen::MatrixXd getQ() { return Q_; }
  inline Eigen::MatrixXd getA() { return A_; }
  inline Eigen::VectorXd getb() { return b_; }

 private:
  BezierCurve bc_;

  int M_;  // number of segments
  int N_;  // order of the polynomial

  std::vector<PolyhedronH> constraints_;
  std::vector<double>          t_;  // time allocation
  Eigen::MatrixXd              init_, end_;

  Eigen::MatrixXd Q_;  // cost matrix
  Eigen::MatrixXd A_;  // constraint matrix
  Eigen::VectorXd q_;  // cost vector
  Eigen::VectorXd b_;  // bound vector
  Eigen::VectorXd x_;  // vector of control points

  Eigen::MatrixXd p2v_;  // position control points to velocity control points
  Eigen::MatrixXd v2a_;  // position control points to acceleration control points
  Eigen::MatrixXd a2j_;  // position control points to jerk control points

 public:
  typedef std::shared_ptr<BezierOpt> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace traj_opt

#endif  // _BEZIER_OPTIMIZER_H_