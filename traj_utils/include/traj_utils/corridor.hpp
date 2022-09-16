/**
 * @file corridor.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief Definition of convex corridors
 * @version 1.0
 * @date 2022-08-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __CORRIDOR_HPP__
#define __CORRIDOR_HPP__

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <vector>

namespace planner {

/**
 * @brief corridor is represented by a convex polyhedron
 * which is a set of hyperplanes, consist of points and normals
 * The first 3 elements are the normal vector, the last 3 elements are the point
 * on that plane.
 * The normal vectors point outward from the polyhedron.
 */
typedef Eigen::Matrix<double, 6, -1> Polyhedron;

/**
 * @brief corridors is a vector of polyhedrons
 */
typedef std::vector<Polyhedron> Corridors;

void convertPolytopeRepresentation(const std::vector<Eigen::Matrix<double, 6, -1>> &c,
                                   std::vector<Eigen::MatrixX4d> &                  h) {
  for (int i = 0; i < static_cast<int>(c.size()); i++) {
    Eigen::MatrixX4d h_i(6, 4);
    for (int j = 0; j < 6; j++) {
      Eigen::Vector3d dir, pos;
      dir = c[i].col(j).head<3>();
      pos = c[i].col(j).tail<3>();
      h_i.row(j) << dir.transpose(), -dir.dot(pos);
    }
    h.push_back(h_i);
  }
}  // namespace planner

#endif  // __CORRIDOR_HPP__