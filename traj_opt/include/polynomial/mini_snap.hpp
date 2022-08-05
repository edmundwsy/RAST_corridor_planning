/**
 * @file mini_snap.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief Solve minimun snap trajectory problem by using iosqp solver
 * @version 1.0
 * @date 2022-08-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef MINI_SNAP_H_
#define MINI_SNAP_H_

#include <stdlib.h>

#include <Eigen/Eigen>
#include <cmath>
#include <vector>

#include "iosqp.hpp"
#include "root_finder.hpp"

#define ORDER       7  // order of polynomial trajectory
#define D_ORDER     4  // order of maximum derivative (4 for minisnap)
#define DIM         3  // number of dimensions in Cspace
#define N_POLYHEDRA 6  // number of polygons in polyhedra

namespace minisnap {

typedef Eigen::Matrix<double, DIM, ORDER + 1> CoefficientMat;
typedef Eigen::SparseMatrix<double>           SparMat;

class Piece {
 private:
  double _duration;
  /** normalized coefficients
   * c0 * 1 + c1 * t + c2 * t^2 + ... + c7 * t^7
   * coeff: [c0, c1, c2, c3, c4, c5, c6, c7]
   */
  CoefficientMat _coeffs;

 public:
  Piece() = default;
  Piece(double duration, const CoefficientMat& coeffs) : _duration(duration), _coeffs(coeffs) {}

  void setup(const double duration) { _duration = duration; }
  void setup(const CoefficientMat& coeffs) { _coeffs = coeffs; }

  inline int                   getDim() const { return DIM; }
  inline int                   getOrder() const { return ORDER; }
  inline double                getDuration() const { return _duration; }
  inline const CoefficientMat& getCoeffs() const { return _coeffs; }

  /**
   * @brief get position at normalized time t
   * @param nt normalized time
   * @return position at normalized time nt
   */
  inline Eigen::Vector3d getPos(double nt) const {
    Eigen::Vector3d pos(0.0, 0.0, 0.0);
    double          t = 1.0;
    for (int i = 0; i <= ORDER; i++) {
      pos += _coeffs.col(i) * t;
      t *= nt;
    }
    return pos;
  }

  /**
   * @brief get velocity at normalized time t
   * @param nt normalized time
   * @return velocity at normalized time nt
   */
  inline Eigen::Vector3d getVel(double nt) const {
    Eigen::Vector3d vel(0.0, 0.0, 0.0);
    double          t = 1.0;
    for (int i = 1; i <= ORDER; i++) {
      vel += i * _coeffs.col(i) * t;
      t *= nt;
    }
    vel = vel / _duration;  // normalize derivative
    return vel;
  }

  /**
   * @brief get acceleration at normalized time t
   * @param nt normalized time
   * @return acceleration at normalized time nt
   */
  inline Eigen::Vector3d getAcc(double nt) const {
    Eigen::Vector3d acc(0.0, 0.0, 0.0);
    double          t = 1.0;
    for (int i = 2; i <= ORDER; i++) {
      acc += i * (i - 1) * _coeffs.col(i) * t;
      t *= nt;
    }
    acc = acc / (_duration * _duration);  // normalize 2nd-order derivative
    return acc;
  }

  /**
   * @brief get jerk at normalized time t
   * @param nt normalized time
   * @return jerk at normalized time nt
   */
  inline Eigen::Vector3d getJrk(double nt) const {
    Eigen::Vector3d jrk(0.0, 0.0, 0.0);
    double          t = 1.0;
    for (int i = 3; i <= ORDER; i++) {
      jrk += i * (i - 1) * (i - 2) * _coeffs.col(i) * t;
      t *= nt;
    }
    jrk = jrk / (_duration * _duration * _duration);  // normalize 3rd-order derivative
    return jrk;
  }

  inline Eigen::MatrixXd getVelCoeffMat() const {
    Eigen::Matrix<double, DIM, ORDER> velCoeffMat;
    for (int i = 1; i <= ORDER; i++) {
      velCoeffMat.col(i - 1) = i * _coeffs.col(i);
    }
    return velCoeffMat;
  }

  inline Eigen::MatrixXd getAccCeoffMat() const {
    Eigen::Matrix<double, DIM, ORDER - 1> accCoeffMat;
    for (int i = 2; i <= ORDER; i++) {
      accCoeffMat.col(i - 2) = i * (i - 1) * _coeffs.col(i);
    }
  }

  // TODO: bug
  inline double getMaxVelRate() const {
    Eigen::MatrixXd velCoeffMat = getVelCoeffMat();
    Eigen::VectorXd coeff       = RootFinder::polySqr(velCoeffMat.row(0).reverse()) +
                            RootFinder::polySqr(velCoeffMat.row(1).reverse()) +
                            RootFinder::polySqr(velCoeffMat.row(2).reverse());
    int N = coeff.size();
    int n = N - 1;
    for (int i = 0; i < N; i++) {
      coeff(i) *= n;
      n--;
    }
    if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON) {
      return 0.0;
    } else {
      double l = -0.0625;
      double r = 1.0625;
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON) {
        l = 0.5 * l;
      }
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON) {
        r = 0.5 * (r + 1.0);
      }
      std::set<double> candidates =
          RootFinder::solvePolynomial(coeff.head(N - 1), l, r, FLT_EPSILON / _duration);
      candidates.insert(0.0);
      candidates.insert(1.0);
      double maxVelRateSqr = -INFINITY;
      double tempNormSqr;
      for (std::set<double>::const_iterator it = candidates.begin(); it != candidates.end(); it++) {
        if (0.0 <= *it && 1.0 >= *it) {
          tempNormSqr   = getVel((*it)).squaredNorm();
          maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
        }
      }
      return sqrt(maxVelRateSqr);
    }
  }

  inline double getMaxAccRate() const {
    Eigen::MatrixXd accCoeffMat = getAccCeoffMat();
    Eigen::VectorXd coeff       = RootFinder::polySqr(accCoeffMat.row(0).reverse()) +
                            RootFinder::polySqr(accCoeffMat.row(1).reverse()) +
                            RootFinder::polySqr(accCoeffMat.row(2).reverse());
    int N = coeff.size();
    int n = N - 1;
    for (int i = 0; i < N; i++) {
      coeff(i) *= n;
      n--;
    }
    if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON) {
      return 0.0;
    } else {
      double l = -0.0625;
      double r = 1.0625;
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON) {
        l = 0.5 * l;
      }
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON) {
        r = 0.5 * (r + 1.0);
      }
      std::set<double> candidates =
          RootFinder::solvePolynomial(coeff.head(N - 1), l, r, FLT_EPSILON / _duration);
      candidates.insert(0.0);
      candidates.insert(1.0);
      double maxAccRateSqr = -INFINITY;
      double tempNormSqr;
      for (std::set<double>::const_iterator it = candidates.begin(); it != candidates.end(); it++) {
        if (0.0 <= *it && 1.0 >= *it) {
          tempNormSqr   = getAcc(*it).squaredNorm();
          maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
        }
      }
      return sqrt(maxAccRateSqr);
    }
  }

};  // class Piece

class Trajectory {
 private:
  typedef std::vector<Piece> Pieces;
  Pieces                     _pieces;
  int                        _n_pieces;

 public:
  Trajectory() = default;
  Trajectory(const std::vector<double>& durations, const std::vector<CoefficientMat>& coeffs) {
    int _n_pieces = durations.size();
    for (int i = 0; i < _n_pieces; i++) {
      _pieces.push_back(Piece(durations[i], coeffs[i]));
    }
  }

  void setDuration(const std::vector<double>& durations) {
    _n_pieces = durations.size();
    _pieces.resize(_n_pieces);
    for (int i = 0; i < _n_pieces; i++) {
      _pieces[i].setup(durations[i]);
    }
  }

  void setCoeffs(const Eigen::VectorXd& x) {
    int M = ORDER + 1;
    for (int i = 0; i < _n_pieces; i++) {
      Eigen::Matrix<double, DIM, ORDER + 1> temp;
      temp << x.segment(DIM * M * i, M).transpose(), x.segment(DIM * M * i + M, M).transpose(),
          x.segment(DIM * M * i + 2 * M, M).transpose();
      _pieces[i].setup(temp);
    }
  }
  void setCoeffs(const std::vector<CoefficientMat>& coeffs) {
    _n_pieces = coeffs.size();
    _pieces.resize(_n_pieces);
    for (int i = 0; i < _n_pieces; i++) {
      _pieces[i].setup(coeffs[i]);
    }
  }

  inline int    getPieceNum() const { return _n_pieces; }
  inline double getDuration() const {
    double duration = 0.0;
    for (int i = 0; i < _n_pieces; i++) {
      duration += _pieces[i].getDuration();
    }
    return duration;
  }

  /**
   * @brief given absolute timestamp, find out piece index and relative timestamp
   * @param t absolute time stamp
   * @param nt normalized time stamp, return nt \in [0, 1)
   * @param idx
   */
  inline void locatePiece(const double& t, double& nt, int& idx) const {
    idx        = _n_pieces;
    double tmp = t;
    for (int i = 0; i < _n_pieces; i++) {
      double Ti = _pieces[i].getDuration();
      if (tmp > Ti) {
        tmp -= Ti;
      } else {
        idx = i;
        nt  = tmp / Ti;
        break;
      }
    }
    /* if t0 is longer than all durations */
    if (idx == _n_pieces) {
      idx = _n_pieces - 1;
      nt  = 1;
    }
  }

  inline Eigen::Vector3d getPos(double t) const {
    double nt;
    int    idx;
    locatePiece(t, nt, idx);
    return _pieces[idx].getPos(nt);
  }

  inline Eigen::Vector3d getVel(double t) const {
    double nt;
    int    idx;
    locatePiece(t, nt, idx);
    return _pieces[idx].getVel(nt);
  }

  inline Eigen::Vector3d getAcc(double t) const {
    double nt;
    int    idx;
    locatePiece(t, nt, idx);
    return _pieces[idx].getAcc(nt);
  }

  inline Eigen::Vector3d getJrk(double t) const {
    double nt;
    int    idx;
    locatePiece(t, nt, idx);
    return _pieces[idx].getJrk(nt);
  }

  double getMaxVelRate() const {
    double max_vel_rate = -INFINITY;
    for (int i = 0; i < _n_pieces; i++) {
      double tmp   = _pieces[i].getMaxVelRate();
      max_vel_rate = max_vel_rate < tmp ? tmp : max_vel_rate;
    }
    return max_vel_rate;
  }

  double getMaxAccRate() const {
    double max_acc_rate = -INFINITY;
    for (int i = 0; i < _n_pieces; i++) {
      double tmp   = _pieces[i].getMaxAccRate();
      max_acc_rate = max_acc_rate < tmp ? tmp : max_acc_rate;
    }
    return max_acc_rate;
  }

  const Piece& operator[](int i) const { return _pieces[i]; }
  Piece&       operator[](int i) { return _pieces[i]; }
};

/***************************************/
/********** Corridor MiniSnap **********/
/***************************************/

/** Jing Chen, Tianbo Liu and Shaojie Shen, "Online generation of collision-free
 * trajectories for quadrotor flight in unknown cluttered environments," 2016
 * IEEE International Conference on Robotics and Automation (ICRA), 2016, pp.
 * 1476-1483, doi: 10.1109/ICRA.2016.7487283.
 */
class CorridorMiniSnap {
 private:
  int                                       N;  // number of pieces
  int                                       n_hyperplanes;
  Eigen::Matrix3d                           _headPVA;  // head's pos, vel, acc
  Eigen::Matrix3d                           _tailPVA;  // tail's pos, vel, acc
  Eigen::MatrixXd                           _Q;
  Eigen::MatrixXd                           _A;
  /**
   * @brief solution of the QP problem
   * _x: DIM * (ORDER + 1) * N vector of coefficients
   * [1st piece] [2nd piece] ... [Nth piece]
   * [i-th piece]: [xxxxxxxx] [yyyyyyyy] [zzzzzzzz]
   */
  Eigen::VectorXd                           _x;
  Eigen::VectorXd                           _ub;
  Eigen::VectorXd                           _lb;
  std::vector<double>                       _timeAlloc;
  std::vector<Eigen::Matrix<double, 6, -1>> _Polygons;

 public:
  CorridorMiniSnap() {}
  ~CorridorMiniSnap() {}
  void reset(const Eigen::Matrix3d&                           head,
             const Eigen::Matrix3d&                           tail,
             const std::vector<double>&                       timeAlloc,
             const std::vector<Eigen::Matrix<double, 6, -1>>& corridors);

  void getCostFunc(const std::vector<double>& factors);
  void getCorridorConstraint();
  void getTransitionConstraint(double delta);
  void getContinuityConstraint();
  void getHeadTailConstraint();

  bool optimize(const std::vector<double>& factors, double delta);
  bool primarySolveQP();
  bool reOptimize();
  // inline bool isCorridorSatisfied(const Eigen::Vector3d & pos, int idx,
  // double t);
  bool   isCorridorSatisfied(Trajectory& traj, double max_vel, double max_acc, double delta);
  void   getTrajectory(Trajectory* traj);
  double getMinimumCost() const;
};

}  // namespace minisnap

#endif  // MINI_SNAP_H_