#ifndef _PATH_NODE_H_
#define _PATH_NODE_H_

#include <path_searching/grid_node.h>
#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

class PathNode;
typedef PathNode* PathNodePtr;

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
};
typedef PathNode* PathNodePtr;

template <typename T>
struct MatrixHash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < (long unsigned int)(matrix.size()); ++i) {
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

#endif  // _PATH_NODE_H_
