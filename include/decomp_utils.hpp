/**
 * @file decomp_utils.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-07-17
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <Eigen/Dense>
#include <Eigen/Eigen>

#include "decomp_ros_msgs/DynPolyhedronArray.h"
#include "decomp_ros_msgs/Polyhedron.h"

namespace decomp_utils {
/**
 * @brief extract corridor from messages
 * @param msgs
 * @return std::vector<Eigen::Matrix<double, 6, -1>>
 */
std::vector<Eigen::Matrix<double, 6, -1>> dynPolyArrayToVector(
    const decomp_ros_msgs::DynPolyhedronArray& msgs) {
  std::vector<Eigen::Matrix<double, 6, -1>> polyhedra;
  polyhedra.reserve(msgs.dyn_polyhedrons.size());
  for (const auto& v : msgs.dyn_polyhedrons) {
    Eigen::MatrixXd polygon;
    polygon.resize(6, v.points.size());
    for (unsigned int i = 0; i < v.points.size(); i++) {
      Eigen::Vector3d p(v.points[i].x, v.points[i].y, v.points[i].z);
      Eigen::Vector3d n(v.normals[i].x, v.normals[i].y, v.normals[i].z);
      polygon.col(i).tail<3>() = p;
      polygon.col(i).head<3>() = n;
    }
    polyhedra.push_back(polygon);
  }
  return polyhedra;
}

/**
 * @brief extract time allocation from message
 * timestamp in msg is the time when uav leaves the corridor
 * @param msgs
 * @return std::vector<double>
 */
std::vector<double> dynPolyArrayToTimeAlloc(
  const decomp_ros_msgs::DynPolyhedronArray& msgs) {
  std::vector<double> time_allocations;
  for (const auto& v : msgs.dyn_polyhedrons) {
    time_allocations.push_back(v.duration);
  }
  return time_allocations;
}

/**
 * @brief extract initial position from msg
 * @param msg
 * @return Eigen::Matrix3d [pos, vel, acc]
 */
Eigen::Matrix3d dynPolyArrayToInitPos(const decomp_ros_msgs::DynPolyhedronArray& msg) {
  Eigen::Matrix3d start;
  start.col(0) << msg.start_pos.x, msg.start_pos.y, msg.start_pos.z;
  start.col(1) << msg.start_vel.x, msg.start_vel.y, msg.start_vel.z;
  start.col(2) << msg.start_acc.x, msg.start_acc.y, msg.start_acc.z;
  return start;
}

/**
 * @brief extract end position from msg
 * @param msg
 * @return Eigen::Matrix3d [pos, vel, acc]
 */
Eigen::Matrix3d dynPolyArrayToEndPos(const decomp_ros_msgs::DynPolyhedronArray& msg) {
  Eigen::Matrix3d end;
  end.col(0) << msg.end_pos.x, msg.end_pos.y, msg.end_pos.z;
  end.col(1) << msg.end_vel.x, msg.end_vel.y, msg.end_vel.z;
  end.col(2) << msg.end_acc.x, msg.end_acc.y, msg.end_acc.z;
  return end;
}
}  // namespace decomp_utils