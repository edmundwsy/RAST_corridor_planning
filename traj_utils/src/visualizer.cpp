/**
 * @file visualizer.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-08-08
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <traj_utils/visualizer.hpp>

namespace visualizer {

/**
 * @brief publish trajectory and color the speed
 *
 * @param start_pos when planning starts
 * @param traj       trajectory to be visualized
 * @param max_vel    maximum velocity to visualize as red line
 */
void Visualizer::visualizeTrajectory(const Eigen::Vector3d&        start_pos,
                                     const polynomial::Trajectory& traj,
                                     double                        max_vel) {
  visualization_msgs::Marker traj_marker;
  traj_marker.header.frame_id    = _frame_id;
  traj_marker.header.stamp       = ros::Time::now();
  traj_marker.type               = visualization_msgs::Marker::LINE_LIST;
  traj_marker.pose.orientation.w = 1.00;
  traj_marker.action             = visualization_msgs::Marker::ADD;
  traj_marker.id                 = 0;
  traj_marker.ns                 = "trajectory";
  traj_marker.color.r            = 0.00;
  traj_marker.color.g            = 0.50;
  traj_marker.color.b            = 1.00;
  traj_marker.color.a            = 1.00;
  traj_marker.scale.x            = 0.10;

  double          T     = 0.05;
  Eigen::Vector3d lastX = traj.getPos(0.0) + start_pos;
  for (double t = T; t < traj.getDuration(); t += T) {
    std_msgs::ColorRGBA c;
    Eigen::Vector3d     jets = jetColorMap(traj.getVel(t).norm() / max_vel);
    c.r                      = jets[0];
    c.g                      = jets[1];
    c.b                      = jets[2];

    geometry_msgs::Point point;
    Eigen::Vector3d      X = traj.getPos(t) + start_pos;
    point.x                = lastX(0);
    point.y                = lastX(1);
    point.z                = lastX(2);
    traj_marker.points.push_back(point);
    traj_marker.colors.push_back(c);
    point.x = X(0);
    point.y = X(1);
    point.z = X(2);
    traj_marker.points.push_back(point);
    traj_marker.colors.push_back(c);
    lastX = X;
  }
  _colorful_traj_pub.publish(traj_marker);
}

/**
 * @brief visualize corridors
 *
 * @param corridors
 * @param pose
 * @param rviz_map_center_locked true if map center is locked
 * @param clear_corridors
 */
void Visualizer::visualizeCorridors(const planner::Corridors& corridors,
                                    const Eigen::Vector3d&    map_pose) {
  displayCorridors(corridors, map_pose, _corridor_pub, _frame_id);
}

void Visualizer::visualizeAstarPath(const std::vector<Eigen::Vector3d>& points) {
  visualization_msgs::Marker pt_marker;
  pt_marker.header.frame_id = _frame_id;
  pt_marker.header.stamp    = ros::Time::now();

  pt_marker.type    = visualization_msgs::Marker::POINTS;
  pt_marker.ns      = "astar_path";
  pt_marker.id      = 0;
  pt_marker.action  = visualization_msgs::Marker::ADD;
  pt_marker.scale.x = 0.2;
  pt_marker.scale.y = 0.2;
  pt_marker.scale.z = 0.2;
  pt_marker.color.a = 1.0;
  pt_marker.color.r = 0.8;
  pt_marker.color.g = 0.3;
  pt_marker.color.b = 0.4;

  visualization_msgs::Marker astar_mkr;
  astar_mkr.header.frame_id = _frame_id;
  astar_mkr.header.stamp    = ros::Time::now();

  astar_mkr.type    = visualization_msgs::Marker::LINE_LIST;
  astar_mkr.action  = visualization_msgs::Marker::ADD;
  astar_mkr.ns      = "astar_path";
  astar_mkr.id      = 1;
  astar_mkr.scale.x = 0.1;
  astar_mkr.scale.y = 0.1;
  astar_mkr.scale.z = 0.1;
  astar_mkr.color.a = 0.1;
  astar_mkr.color.r = 0.9;
  astar_mkr.color.g = 0.2;
  astar_mkr.color.b = 1.0;

  astar_mkr.pose.orientation.w = 1.0;

  Eigen::Vector3d last;
  bool            first = true;
  for (const auto& point : points) {
    if (first) {
      first = false;
      last  = point;
      continue;
    }
    geometry_msgs::Point p;
    p.x = last.x();
    p.y = last.y();
    p.z = last.z();
    astar_mkr.points.push_back(p);
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    astar_mkr.points.push_back(p);
    pt_marker.points.push_back(p);
    last = point;
  }

  // _astar_path_pub.publish(astar_mkr);
  _astar_path_pub.publish(pt_marker);
}

/**
 * @brief visualize start and goal points
 *
 * @param center
 * @param radius
 * @param sg
 */
void Visualizer::visualizeStartGoal(const Eigen::Vector3d& center, int sg) {
  visualization_msgs::Marker sphereMarkers, sphereDeleter;
  float                      radius = 0.1;

  sphereMarkers.id                 = sg;
  sphereMarkers.type               = visualization_msgs::Marker::SPHERE_LIST;
  sphereMarkers.header.stamp       = ros::Time::now();
  sphereMarkers.header.frame_id    = _frame_id;
  sphereMarkers.pose.orientation.w = 1.00;
  sphereMarkers.action             = visualization_msgs::Marker::ADD;
  sphereMarkers.ns                 = "StartGoal";
  sphereMarkers.color.r            = 1.00;
  sphereMarkers.color.g            = 0.00;
  sphereMarkers.color.b            = 0.00;
  sphereMarkers.color.a            = 1.00;
  sphereMarkers.scale.x            = radius * 2.0;
  sphereMarkers.scale.y            = radius * 2.0;
  sphereMarkers.scale.z            = radius * 2.0;

  sphereDeleter        = sphereMarkers;
  sphereDeleter.action = visualization_msgs::Marker::DELETEALL;

  geometry_msgs::Point point;
  point.x = center(0);
  point.y = center(1);
  point.z = center(2);
  sphereMarkers.points.push_back(point);

  if (sg == 0) {
    _start_goal_pub.publish(sphereDeleter);
    ros::Duration(1.0e-9).sleep();
    sphereMarkers.header.stamp = ros::Time::now();
  }
  _start_goal_pub.publish(sphereMarkers);
}
}  // namespace visualizer
