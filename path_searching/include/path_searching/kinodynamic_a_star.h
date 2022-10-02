/**
 * @file kinodynamic_a_star.h
 * @author Bolei Zhou
 * @brief This file is based on Fast-Planner (github.com/HKUST-Aerial-Robotics/Fast-Planner)
 * @version 1.0
 * @date 2022-09-29
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>

#include <Eigen/Core>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>

#define inf 1 >> 30

class PathNode : public GridNode {
 public:
  /* -------------------- */
  PathNode*                   parent_{nullptr};
  Eigen::Vector3d             input;
  Eigen::Matrix<double, 6, 1> state;
  double                      duration;
  double                      time;  // dyn
  int                         time_idx;

  /* -------------------- */
  PathNode() {
    setParent(NULL);
    setNodeState(NOT_EXPAND);
  }
  ~PathNode() {}
  void             setParent(PathNode* parent) { parent_ = parent; }
  inline PathNode* getParent() const { return parent_; }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode* PathNodePtr;

template <typename T>
struct MatrixHash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
 private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, MatrixHash<Eigen::Vector3i>> data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, MatrixHash<Eigen::Vector4i>> data_4d_;

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3i idx, PathNodePtr node) { data_3d_.insert(std::make_pair(idx, node)); }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node) {
    data_4d_.insert(std::make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }
};

class KinodynamicAstar : public AStar {
 private:
  /* ---------- parameter ---------- */
  /* Derived from Base Class */
  // int    rounds_{0};
  // double resolution_, inv_resolution_;
  // double tie_breaker_;
  // Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
  // Eigen::Vector3d map_center_;  // map center

  // GridMap::Ptr             grid_map_;
  // GridNodePtr ***          GridNodeMap_;
  std::vector<PathNodePtr> node_path_;

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;

  /* search */
  bool   optimistic_;
  int    allocate_num_;  // number of nodes to be allocated in path_node_pool_
  int    check_num_;
  double max_tau_, init_max_tau_;
  double max_vel_, max_acc_;
  double w_time_, horizon_, lambda_heu_;
  double time_resolution_, inv_time_resolution_;

  double time_origin_;

  /* ---------- main data structure ---------- */
  int                      use_node_num_, iter_num_;
  std::vector<PathNodePtr> path_node_pool_;
  NodeHashTable            expanded_nodes_;

  /* ---------- record data ---------- */
  bool                        is_shot_succ_ = false;
  bool                        has_path_     = false;
  double                      t_shot_;
  Eigen::Vector3d             start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
  Eigen::MatrixXd             coef_shot_;

  /* helper */
  int             timeToIndex(double time);
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  // void            retrievePath(PathNodePtr end_node);  // in Base class

  /* shot trajectory */
  std::vector<double> cubic(double a, double b, double c, double d);
  std::vector<double> quartic(double a, double b, double c, double d, double e);
  bool   computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);

  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                    Eigen::Matrix<double, 6, 1>& state1,
                    Eigen::Vector3d              um,
                    double                       tau);

 public:
  KinodynamicAstar() {}
  ~KinodynamicAstar();
  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };
  typedef std::shared_ptr<KinodynamicAstar> Ptr;
  
  /* main API */
  void setParam(ros::NodeHandle& nh);
  void init(const Eigen::Vector3d& map_center, const Eigen::Vector3i& map_size);
  void reset();

  ASTAR_RET search(Eigen::Vector3d start_pt,
                   Eigen::Vector3d start_vel,
                   Eigen::Vector3d start_acc,
                   Eigen::Vector3d end_pt,
                   Eigen::Vector3d end_vel,
                   bool            init,
                   bool            dynamic    = false,
                   double          time_start = -1.0);

  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);

  void getSamples(double&                       ts,
                  std::vector<Eigen::Vector3d>& point_set,
                  std::vector<Eigen::Vector3d>& start_end_derivatives);

  std::vector<PathNodePtr> getVisitedNodes();
};

#endif  // _KINODYNAMIC_ASTAR_H
