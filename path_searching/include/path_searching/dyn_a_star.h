#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <plan_env/grid_map.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <queue>
#include <vector>

constexpr double inf = 1 >> 20;
class GridNode;
typedef GridNode *GridNodePtr;
enum ASTAR_RET { SUCCESS, INIT_ERR, SEARCH_ERR };
enum NODE_STATE { IN_CLOSE_SET = 1, IN_OPEN_SET = 2, NOT_EXPAND = 3 };

class GridNode {
 private:
  int             rounds_{0};  // Distinguish every call
  double          g_score_{inf}, f_score_{inf};
  GridNodePtr     parent_{nullptr};
  enum NODE_STATE node_state_ { NOT_EXPAND };
  Eigen::Vector3i index_;

 public:
  void setNodeState(const enum NODE_STATE &state) { node_state_ = state; }
  void setIndex(const Eigen::Vector3i &idx) { index_ = idx; }
  void setRounds(const int &r) { rounds_ = r; }
  void setGScore(const double &g) { g_score_ = g; }
  void setFScore(const double &f) { f_score_ = f; }
  void setState(const enum NODE_STATE &s) { node_state_ = s; }
  void setParent(GridNode *p) { parent_ = p; }

  inline const double &getFScore() { return this->f_score_; }
  inline const double &getGScore() { return this->g_score_; }
  inline const int &   getRounds() { return this->rounds_; }

  inline const enum NODE_STATE &getNodeState() { return this->node_state_; }

  inline const GridNodePtr getParent() const { return this->parent_; }

  inline int   getIndex(int i) const { return index_(i); }
  inline Eigen::Vector3i getIndex() const { return index_; }
};

class NodeComparator {
 public:
  bool operator()(GridNodePtr node1, GridNodePtr node2) {
    return node1->getFScore() > node2->getFScore();
  }
};

class AStar {
 private:
  /* ----- parameters ----- */
  int          rounds_{0};
  double       step_size_, inv_step_size_;
  const double tie_breaker_ = 1.0 + 1.0 / 10000;

  /* ----- data ----- */
  Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
  Eigen::Vector3d center_;

  GridMap::Ptr             grid_map_;
  GridNodePtr ***          GridNodeMap_;
  std::vector<GridNodePtr> gridPath_;

  std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

  /* ----- function ----- */

  bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt,
                                             const Eigen::Vector3d end_pt,
                                             Eigen::Vector3i &     start_idx,
                                             Eigen::Vector3i &     end_idx);

  virtual double getHeu(GridNodePtr node1, GridNodePtr node2) {
    return tie_breaker_ * getDiagHeu(node1, node2);
  }
  inline void coord2gridIndexFast(
      const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);
  inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
  inline bool            Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

  inline int checkOccupancy(const Eigen::Vector3d &pos) {
    return grid_map_->getInflateOccupancy(pos);
  }

  std::vector<GridNodePtr> retrievePath(GridNodePtr current);

 public:
  typedef std::shared_ptr<AStar> Ptr;

  AStar() {}
  ~AStar();

  void   initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);
  double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
  double getManhHeu(GridNodePtr node1, GridNodePtr node2);
  double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
  double getTieBreaker() { return tie_breaker_; }

  ASTAR_RET search(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

  std::vector<Eigen::Vector3d> getPath();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/* ----- inline functions ----- */

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const {
  return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
}

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const {
  idx =
      ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

  if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) ||
      idx(2) < 0 || idx(2) >= POOL_SIZE_(2)) {
    ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
    return false;
  }

  return true;
}

class AStarManhHeu : public AStar {
  virtual double getHeu(GridNodePtr node1, GridNodePtr node2) {
    return getTieBreaker() * getManhHeu(node1, node2);
  }
};

#endif  // _DYN_A_STAR_H_
