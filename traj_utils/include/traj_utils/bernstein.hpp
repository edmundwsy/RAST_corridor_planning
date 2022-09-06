/**
 * @file bernstein.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-09-06
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _BERNSTEIN_H_
#define _BERNSTEIN_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>

namespace planner {

const int ORDER = 4;  // order of Bezier curve, default 4
const int DIM   = 3;  // dimension of the trajectory

class BernsteinPiece {
 private:
  /** control points for Bernstein with different dimensions.
   * Each row represents one single control point
   * The dimension is determined by column number
   * e.g. Bernstein with N points in 3D space -> Nx3 matrix */
  Eigen::MatrixXd cpts_;     // control points
  Eigen::MatrixXd A_;        // coefficient matrix
  int             N_;        // order
  double          t0_, tf_;  // time interval
  double          t_;        // duration

 public:
  BernsteinPiece() = default;
  BernsteinPiece(const Eigen::MatrixXd &cpts, const double &t0, const double &tf) {
    cpts_ = cpts;
    N_    = ORDER;
    t0_   = t0;
    tf_   = tf;
    t_    = tf_ - t0_;
    assert(cpts_.rows() == N_ + 1);  // 4th order curve has 5 control points
    calcCoeffMat();
  }
  BernsteinPiece(const Eigen::MatrixXd &cpts, const double &t) {
    cpts_ = cpts;
    N_    = ORDER;
    t0_   = 0;
    tf_   = t;
    t_    = t;
    assert(cpts_.rows() == N_ + 1);
    calcCoeffMat();
  }

  ~BernsteinPiece() {}

  inline int    getDim() { return DIM; }
  inline int    getOrder() { return N_; }
  inline double getStartTime() { return t0_; }
  inline double getEndTime() { return tf_; }
  inline double getDuration() { return tf_ - t0_; }

  Eigen::Vector3d getPos(double t) const;
  Eigen::Vector3d getVel(double t) const;
  Eigen::Vector3d getAcc(double t) const;
  Eigen::Vector3d getJrk(double t) const;

  Eigen::MatrixXd getPosCtrlPts() const { return cpts_; }
  Eigen::MatrixXd getVelCtrlPts() const;
  Eigen::MatrixXd getAccCtrlPts() const;

  double getMaxVelRate() const;
  double getMaxAccRate() const;

  void calcCoeffMat();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}

class Bezier {
 private:
  typedef std::vector<BernsteinPiece> Pieces;
  Pieces pieces_;
  Eigen::MatrixXd     cpts_;  // control points
  int                 N_;     // order
  int                 M_;     // number of pieces
  double              T_;     // total time
  std::vector<double> t_;     // time interval for each piece

 public:
  Bezier() {}
  Bezier(const int &order, const double &time);
  ~Bezier();

  /* get & set basic info */
  void   setOrder(const int &order) { N_ = order; }
  void   setTime(const std::vector<double> &t) { t_ = t; }
  int    getOrder() { return N_ }
  double getDuration() { return T_; }

  inline Eigen::Vector3d getPos(double t);
  inline Eigen::Vector3d getVel(double t);
  inline Eigen::Vector3d getAcc(double t);

  double getMaxVelRate() const;
  double getMaxAccRate() const;

  const BernsteinPiece& operator[](int i) const { return pieces_[i]; }
  BernsteinPiece& operator[](int i) { return pieces_[i]; }

  typedef std::shared_ptr<Bezier> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planner

#endif