/**
 * @file vis_bezier_opt.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-09-11
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

#include "bernstein/bezier_optimizer.hpp"
#include "traj_utils/bernstein.hpp"
#include "traj_utils/visualizer.hpp"

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

Bernstein::Bezier::Ptr      _ptr_bezier;
traj_opt::BezierOpt::Ptr    _ptr_bezier_opt;
visualizer::Visualizer::Ptr _vis;

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
std::vector<Eigen::MatrixX4d> getRandomCorridors(int n, const std::vector<Eigen::Vector3d> &wps) {
  double          l_margin = 0.5;
  double          w_margin = 3.0;
  double          h        = 2.0;
  Eigen::Vector3d up(0.0, 0.0, 1.0);

  std::vector<Eigen::MatrixX4d> crds;
  for (int i = 0; i < n; i++) {
    Eigen::MatrixX4d crd;
    crd.resize(6, Eigen::NoChange);
    Eigen::Vector3d srt = wps[i];
    Eigen::Vector3d end = wps[i + 1];
    Eigen::Vector3d mdl = (srt + end) / 2;

    Eigen::Vector3d lvec = end - srt;
    lvec                 = lvec / lvec.norm();
    Eigen::Vector3d wvec = up.cross(lvec);

    Eigen::Vector3d dir;
    Eigen::Vector3d pos;

    dir = -lvec;
    pos = srt - lvec * l_margin;
    crd.row(0) << dir.transpose(), -pos.dot(dir);

    dir = lvec;
    pos = end + lvec * l_margin;
    crd.row(1) << dir.transpose(), -pos.dot(dir);

    dir = up;
    pos = mdl + up * h;
    crd.row(2) << dir.transpose(), -pos.dot(dir);

    dir = -up;
    pos = mdl - up * h;
    crd.row(3) << dir.transpose(), -pos.dot(dir);

    dir = wvec;
    pos = mdl + wvec * w_margin;
    crd.row(4) << dir.transpose(), -pos.dot(dir);

    dir = -wvec;
    pos = mdl - wvec * w_margin;
    crd.row(5) << dir.transpose(), -pos.dot(dir);
    std::cout << i << std::endl << crd << std::endl;
    crds.push_back(crd);
  }
  return crds;
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
  init_state.row(0) = _start_pos;
  init_state.row(1) = _start_vel;
  init_state.row(2) = _start_acc;
  goal_state.row(0) = _end_pos;
  goal_state.row(1) = _end_vel;
  goal_state.row(2) = _end_acc;

  std::vector<double> time_alloc = getRandomTimeAlloc(n);

  std::cout << "time_alloc = " << std::endl;
  for (auto it : time_alloc) {
    std::cout << it << " ";
  }
  std::cout << std::endl;

  std::vector<Eigen::Vector3d>  wpts      = getRandomWaypoint(n, _start_pos, _end_pos);
  std::vector<Eigen::MatrixX4d> corridors = getRandomCorridors(n, wpts);
  std::cout << "goal" << goal_state << std::endl;
  _ptr_bezier_opt.reset(new traj_opt::BezierOpt());
  _ptr_bezier_opt->setup(init_state, goal_state, time_alloc, corridors);
  ros::Time tic    = ros::Time::now();
  bool      status = _ptr_bezier_opt->optimize();
  ros::Time toc    = ros::Time::now();
  double    t_comp = (toc - tic).toSec() * 1000;
  ROS_INFO("Trajectory optimized in %f milliseconds", t_comp);

  if (status) {
    _ptr_bezier = _ptr_bezier_opt->getOptBezier();
  } else {
    ROS_ERROR("Trajectory optimization failed");
    Eigen::Vector3d zero(0.0, 0.0, 0.0);
    ROS_INFO("Visualizing corridors");
    _vis->visualizeCorridors(corridors, zero);
    return;
  }

  double duration = 0.0;
  for (auto it = time_alloc.begin(); it != time_alloc.end(); it++) {
    duration += *it;
  }
  ROS_INFO("Total duration = %f", duration);
  double max_vel = _ptr_bezier->getMaxVelRate();
  ROS_INFO("Max vel rate = %f", max_vel);

  Eigen::Vector3d zero(0.0, 0.0, 0.0);
  ROS_INFO("Visualizing corridors");
  _vis->visualizeCorridors(corridors, zero);
  ROS_INFO("Visualizing waypoints");
  _vis->visualizeBezierCurve(zero, *_ptr_bezier, 4.0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_bezier_node");
  ros::NodeHandle nh("~");

  _vis = std::shared_ptr<visualizer::Visualizer>(new visualizer::Visualizer(nh));
  ros::Subscriber click_sub = nh.subscribe("/move_base_simple/goal", 1, clickCallback);

  _vis_pub   = nh.advertise<visualization_msgs::Marker>("trajectory", 10);
  _text_pub  = nh.advertise<visualization_msgs::Marker>("text", 10);
  _route_pub = nh.advertise<visualization_msgs::Marker>("route", 10);
  ros::spin();
  return 0;
}