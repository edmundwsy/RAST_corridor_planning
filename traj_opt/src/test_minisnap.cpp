/**
 * @file test_minisnap.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-08-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <iostream>
#include <random>
#include <vector>

#include "traj_utils/visualizer.hpp"
#include "polynomial/mini_snap.h"

/* initialize random seed */
std::random_device                     rd;
std::default_random_engine             eng(rd());
std::uniform_real_distribution<double> _x_rand(-1.0, 1.0);
std::uniform_real_distribution<double> _z_rand(0.0, 3.0);
std::uniform_real_distribution<double> _v_rand(0.0, 5.0);
std::uniform_real_distribution<double> _t_rand(0.0, 2.0);  // time interval

Eigen::Vector3d _start_pos(0.0, 0.0, 0.0);
Eigen::Vector3d _start_vel(0.0, 0.0, 0.0);
Eigen::Vector3d _start_acc(0.0, 0.0, 0.0);
Eigen::Vector3d _end_pos(0.0, 0.0, 0.0);
Eigen::Vector3d _end_vel(0.0, 0.0, 0.0);
Eigen::Vector3d _end_acc(0.0, 0.0, 0.0);

ros::Publisher _vis_pub, _route_pub, _text_pub, _wpt_pub;

polynomial::Trajectory       _traj;
polynomial::CorridorMiniSnap _optimizer;
visualizer::Visualizer::Ptr       _vis;

/**
 * @brief optimize trajectory
 *
 * @param init
 * @param goal
 * @param t_alloc
 * @param corridors
 * @return true
 * @return false
 */
bool Optimize(const Eigen::Matrix3d &                          init,
              const Eigen::Matrix3d &                          goal,
              const std::vector<double> &                      t_alloc,
              const std::vector<Eigen::Matrix<double, 6, -1>> &corridors) {
  bool                is_solved = false;
  std::vector<double> factors   = {0.0, 0.0, 0.0, 0.0, 1.0};

  /* initial guess */
  _optimizer.reset(init, goal, t_alloc, corridors);
  try {
    is_solved = _optimizer.optimize(0.0);

  } catch (int e) {
    ROS_ERROR("Optimization failed!");
    return false;
  }

  if (is_solved) {
    _optimizer.getTrajectory(&_traj);
  } else {
    ROS_ERROR("Optimization failed!");
    return false;
  }

  /* reoptimization */
  int i = 0, I = 10;
  while (!_optimizer.isCorridorSatisfied(_traj, 10.0, 10.0, 0) && i++ < I) {
    try {
      is_solved = _optimizer.reOptimize();
    } catch (int e) {
      ROS_ERROR("Re-optimization failed!");
      return false;
    }

    if (is_solved) {
      _optimizer.getTrajectory(&_traj);
    } else {
      ROS_ERROR("Re-optimization failed!");
      return false;
    }
  }
  return true;
}

/**
 * @brief randomly generate time allocations
 *
 * @param n
 * @return std::vector<double>
 */
std::vector<double> getRandomTimeAlloc(int n) {
  std::vector<double> time_alloc(n);
  for (int i = 0; i < n; i++) {
    time_alloc[i] = _t_rand(eng);
  }
  return time_alloc;
}

/**
 * @brief Get the Random Waypoint object
 *
 * @param n
 * @param srt
 * @param end
 * @return std::vector<Eigen::Vector3d>
 */
std::vector<Eigen::Vector3d> getRandomWaypoint(int                    n,
                                               const Eigen::Vector3d &srt,
                                               const Eigen::Vector3d &end) {
  std::vector<Eigen::Vector3d> waypoints(n + 1);
  waypoints[0] = srt;
  waypoints[n] = end;
  for (int i = 1; i < n; i++) {
    waypoints[i] = srt + (end - srt) * i / n;
    waypoints[i][0] += _x_rand(eng);
    waypoints[i][1] += _x_rand(eng);
    waypoints[i][2] = _z_rand(eng);
  }
  return waypoints;
}

std::vector<Eigen::Matrix<double, 6, -1>> getRandomCorridors(
    int n, const std::vector<Eigen::Vector3d> &wps) {
  double          l_margin = 0.5;
  double          w_margin = 3.0;
  double          h        = 2.0;
  Eigen::Vector3d up(0.0, 0.0, 1.0);

  std::vector<Eigen::Matrix<double, 6, -1>> crds;
  for (int i = 0; i < n; i++) {
    Eigen::Matrix<double, 6, 6> crd;
    Eigen::Vector3d             srt = wps[i];
    Eigen::Vector3d             end = wps[i + 1];
    Eigen::Vector3d             mdl = (srt + end) / 2;

    Eigen::Vector3d lvec = end - srt;
    lvec                 = lvec / lvec.norm();
    Eigen::Vector3d wvec = up.cross(lvec);

    crd.col(0).head<3>() = -lvec;
    crd.col(0).tail<3>() = srt - lvec * l_margin;
    crd.col(1).head<3>() = lvec;
    crd.col(1).tail<3>() = end + lvec * l_margin;
    crd.col(2).head<3>() = up;
    crd.col(2).tail<3>() = mdl + up * h;
    crd.col(3).head<3>() = -up;
    crd.col(3).tail<3>() = mdl - up * h;
    crd.col(4).head<3>() = wvec;
    crd.col(4).tail<3>() = mdl + wvec * w_margin;
    crd.col(5).head<3>() = -wvec;
    crd.col(5).tail<3>() = mdl - wvec * w_margin;
    crds.push_back(crd);
  }
  return crds;
}

inline Eigen::Vector3d jetColorMap(double a) {
  double          s = a * 4;
  Eigen::Vector3d c;  // [r, g, b]
  switch (static_cast<int>(floor(s))) {
    case 0:
      c << 0, 0, s;
      break;
    case 1:
      c << 0, s - 1, 1;
      break;
    case 2:
      c << s - 2, 1, 3 - s;
      break;
    case 3:
      c << 1, 4 - s, 0;
      break;
    default:
      c << 1, 0, 0;
      break;
  }
  return c;
}

// TODO: move it to traj_utils visualizer
void visualizeCorridor() {}

void visualizeTraj(const polynomial::Trajectory &        traj,
                   const std::vector<Eigen::Vector3d> &route,
                   ros::Time                           timeStamp,
                   double                              compT,
                   double                              maxV,
                   double                              totalT) {
  visualization_msgs::Marker routeMarker, wayPointsMarker, trajMarker;

  routeMarker.id                 = 0;
  routeMarker.type               = visualization_msgs::Marker::LINE_LIST;
  routeMarker.header.stamp       = timeStamp;
  routeMarker.header.frame_id    = "world";
  routeMarker.pose.orientation.w = 1.00;
  routeMarker.action             = visualization_msgs::Marker::ADD;
  routeMarker.ns                 = "route";
  routeMarker.color.r            = 1.00;
  routeMarker.color.g            = 0.00;
  routeMarker.color.b            = 0.00;
  routeMarker.color.a            = 1.00;
  routeMarker.scale.x            = 0.05;

  wayPointsMarker         = routeMarker;
  wayPointsMarker.type    = visualization_msgs::Marker::SPHERE_LIST;
  wayPointsMarker.ns      = "waypoints";
  wayPointsMarker.color.r = 1.00;
  wayPointsMarker.color.g = 0.00;
  wayPointsMarker.color.b = 0.00;
  wayPointsMarker.scale.x = 0.20;
  wayPointsMarker.scale.y = 0.20;
  wayPointsMarker.scale.z = 0.20;

  trajMarker                 = routeMarker;
  trajMarker.header.frame_id = "world";
  trajMarker.id              = 0;
  trajMarker.ns              = "trajectory";
  trajMarker.color.r         = 0.00;
  trajMarker.color.g         = 0.50;
  trajMarker.color.b         = 1.00;
  trajMarker.scale.x         = 0.10;

  if (route.size() > 0) {
    bool            first = true;
    Eigen::Vector3d last;
    for (auto it : route) {
      if (first) {
        first = false;
        last  = it;
        continue;
      }
      geometry_msgs::Point point;

      point.x = last(0);
      point.y = last(1);
      point.z = last(2);
      routeMarker.points.push_back(point);
      point.x = it(0);
      point.y = it(1);
      point.z = it(2);
      routeMarker.points.push_back(point);
      last = it;
    }

    _route_pub.publish(routeMarker);
  }

  if (traj.getPieceNum() > 0) {
    double          T     = 0.05;
    double          v_max = 0.0;
    Eigen::Vector3d lastX = traj.getPos(0.0);
    for (double t = T; t < traj.getDuration(); t += T) {
      std_msgs::ColorRGBA c;
      Eigen::Vector3d     V    = traj.getVel(t);
      Eigen::Vector3d     jets = jetColorMap(V.norm() / maxV);
      c.r                      = jets[0];
      c.g                      = jets[1];
      c.b                      = jets[2];

      geometry_msgs::Point point;
      Eigen::Vector3d      X = traj.getPos(t);
      point.x                = lastX(0);
      point.y                = lastX(1);
      point.z                = lastX(2);
      trajMarker.points.push_back(point);
      trajMarker.colors.push_back(c);
      point.x = X(0);
      point.y = X(1);
      point.z = X(2);
      trajMarker.points.push_back(point);
      trajMarker.colors.push_back(c);
      lastX = X;
      // TEST IF VELOCITY IS CORRECT
      Eigen::Vector3d A = traj.getAcc(t);
      std::cout << "t: " << t << " X: " << X.transpose() << " V: " << V.transpose()
                << " A: " << A.transpose() << std::endl;
      v_max = std::max(v_max, V.norm());
    }
    ROS_INFO("v_max: %f", v_max);
    _vis_pub.publish(trajMarker);
  }

  if (traj.getPieceNum() > 0) {
    visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = "world";
    textMarker.header.stamp    = timeStamp;
    textMarker.ns              = "text";
    textMarker.id              = 1;
    textMarker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.action          = visualization_msgs::Marker::ADD;

    textMarker.pose.position.x    = -9;
    textMarker.pose.position.y    = 0.0;
    textMarker.pose.position.z    = 6.0;
    textMarker.pose.orientation.x = 0.0;
    textMarker.pose.orientation.y = 0.0;
    textMarker.pose.orientation.z = 0.0;
    textMarker.pose.orientation.w = 1.0;
    textMarker.scale.x            = 1.0;
    textMarker.scale.y            = 1.0;
    textMarker.scale.z            = 1.0;
    textMarker.color.r            = 1.0;
    textMarker.color.g            = 0.0;
    textMarker.color.b            = 0.0;
    textMarker.color.a            = 1.0;
    textMarker.text               = "Comp: ";
    textMarker.text += std::to_string(static_cast<int>(compT));
    textMarker.text += ".";
    textMarker.text += std::to_string(static_cast<int>(compT * 10) % 10);
    textMarker.text += "ms\n";
    textMarker.text += "Max speed: ";
    textMarker.text += std::to_string(static_cast<int>(maxV));
    textMarker.text += ".";
    textMarker.text += std::to_string(static_cast<int>(maxV * 100) % 100);
    textMarker.text += "m/s\n";
    textMarker.text += "Total time: ";
    textMarker.text += std::to_string(static_cast<int>(totalT));
    textMarker.text += ".";
    textMarker.text += std::to_string(static_cast<int>(totalT * 100) % 100);
    textMarker.text += "s\n";
    _text_pub.publish(textMarker);
  }
}

void clickCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  _end_pos(0) = msg->pose.position.x;
  _end_pos(1) = msg->pose.position.y;
  _end_pos(2) = _z_rand(eng);

  Eigen::Quaterniond q;
  q.x() = msg->pose.orientation.x;
  q.y() = msg->pose.orientation.y;
  q.z() = msg->pose.orientation.z;
  q.w() = msg->pose.orientation.w;
  Eigen::Vector3d axis(1.0, 0.0, 0.0);
  _end_vel = q * axis * _v_rand(eng);

  int n = 2 + std::rand() % 10;
  ROS_INFO("number of pieces = %d", n);

  Eigen::Matrix3d init_state;
  Eigen::Matrix3d goal_state;
  init_state.col(0) = _start_pos;
  init_state.col(1) = _start_vel;
  init_state.col(2) = _start_acc;
  goal_state.col(0) = _end_pos;
  goal_state.col(1) = _end_vel;
  goal_state.col(2) = _end_acc;

  std::vector<double> time_alloc = getRandomTimeAlloc(n);

  std::cout << "time_alloc = " << std::endl;
  for (auto it : time_alloc) {
    std::cout << it << " ";
  }
  std::cout << std::endl;

  std::vector<Eigen::Vector3d>              wpts      = getRandomWaypoint(n, _start_pos, _end_pos);
  std::vector<Eigen::Matrix<double, 6, -1>> corridors = getRandomCorridors(n, wpts);
  std::cout << "goal" << goal_state << std::endl;
  ros::Time tic    = ros::Time::now();
  bool      status = Optimize(init_state, goal_state, time_alloc, corridors);
  ros::Time toc    = ros::Time::now();
  double    t_comp = (toc - tic).toSec() * 1000;
  ROS_INFO("Trajectory optimized in %f seconds", t_comp);

  double duration = 0.0;
  for (auto it = time_alloc.begin(); it != time_alloc.end(); it++) {
    duration += *it;
  }
  double max_vel = _traj.getMaxVelRate();

  visualizeTraj(_traj, wpts, toc, t_comp, max_vel, duration);
  Eigen::Vector3d zero(0.0, 0.0, 0.0);
  _vis->visualizeCorridors(corridors, zero);
  ROS_INFO("Trajectory and corridor visualized");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_minisnap_node");
  ros::NodeHandle nh("~");

  _vis = std::shared_ptr<visualizer::Visualizer>(new visualizer::Visualizer(nh));
  ros::Subscriber click_sub = nh.subscribe("/move_base_simple/goal", 1, clickCallback);

  _vis_pub   = nh.advertise<visualization_msgs::Marker>("trajectory", 10);
  _text_pub  = nh.advertise<visualization_msgs::Marker>("text", 10);
  _route_pub = nh.advertise<visualization_msgs::Marker>("route", 10);
  ros::spin();
  return 0;
}