/* ----------------------------------------------------------------------------
 * Copyright 2023, Siyuan Wu, Autonomous Multi-Robot Lab
 * Delft University of Technology
 * All Rights Reserved
 * Part of visualization are based on https://github.com/mit-acl/mader
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <ros/ros.h>
#include <traj_utils/BezierTraj.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>
#include "traj_utils/bernstein.hpp"

double MAX_VEL  = 3.0;
double SCALE    = 0.1;
double MAX_ACC  = 5.0;
int    AGENT_ID = 0;
int    N_AGENTS = 1;

ros::Publisher colored_traj_pub_;

struct state {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

typedef std::vector<state> trajectory;

/**
 * @brief get color from jet colormap
 *
 * @param v current value
 * @param vmin min value
 * @param vmax max value
 * @return std_msgs::ColorRGBA
 */
std_msgs::ColorRGBA getColorJet(double v, double vmin, double vmax) {
  std_msgs::ColorRGBA c;
  c.r = 1;
  c.g = 1;
  c.b = 1;
  c.a = 1;
  // white
  double dv;

  if (v < vmin) v = vmin;
  if (v > vmax) v = vmax;
  dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv)) {
    c.r = 0;
    c.g = 4 * (v - vmin) / dv;
  } else if (v < (vmin + 0.5 * dv)) {
    c.r = 0;
    c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  } else if (v < (vmin + 0.75 * dv)) {
    c.r = 4 * (v - vmin - 0.5 * dv) / dv;
    c.b = 0;
  } else {
    c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    c.b = 0;
  }

  return (c);
}

visualization_msgs::MarkerArray trajectory2ColoredMarkerArray(const trajectory& data,
                                                              double            max_value,
                                                              int               increm,
                                                              std::string       ns,
                                                              double            scale,
                                                              std::string       color_type,
                                                              int               id_agent,
                                                              int               n_agents) {
  visualization_msgs::MarkerArray marker_array;

  if (data.size() == 0) {
    return marker_array;
  }
  geometry_msgs::Point p_last;
  p_last.x = data[0].pos(0);
  p_last.y = data[0].pos(1);
  p_last.z = data[0].pos(2);

  increm = (increm < 1.0) ? 1 : increm;

  int j = 0;
  for (int i = 0; i < data.size(); i = i + increm) {
    double                     vel = data[i].vel.norm();
    visualization_msgs::Marker m;
    m.type            = visualization_msgs::Marker::ARROW;
    m.header.frame_id = "world";
    m.header.stamp    = ros::Time::now();
    m.ns              = ns;
    m.action          = visualization_msgs::Marker::ADD;
    m.id              = j;
    if (color_type == "vel") {
      m.color = getColorJet(vel, 0, max_value);  // note that par_.v_max is per axis!
    } else if (color_type == "time") {
      m.color = getColorJet(i, 0, data.size());  // note that par_.v_max is per axis!
    } else {
      m.color = getColorJet(id_agent, 0, n_agents);  // note that par_.v_max is per axis!
    }
    m.scale.x = scale;
    m.scale.y = 0.0000001;  // rviz complains if not
    m.scale.z = 0.0000001;  // rviz complains if not

    m.pose.orientation.w = 1.0;
    geometry_msgs::Point p;
    p.x = data[i].pos(0);
    p.y = data[i].pos(1);
    p.z = data[i].pos(2);
    m.points.push_back(p_last);
    m.points.push_back(p);
    p_last = p;
    marker_array.markers.push_back(m);
    j = j + 1;
  }
  return marker_array;
}

/**
 * @brief callback function for bezier trajectory
 *
 * @param msg bezier trajectory
 */
void BezierCallback(const traj_utils::BezierTrajConstPtr& msg) {
  int id      = msg->drone_id;
  int N       = msg->order;
  int n_piece = msg->duration.size();  // number of pieces
  int R       = n_piece * (N + 1);     // number of control points
  //
  ros::Time t_start = msg->start_time;
  ros::Time t_pub   = msg->pub_time;

  double              duration = 0.0;
  std::vector<double> time_alloc;
  for (auto it = msg->duration.begin(); it != msg->duration.end(); ++it) {
    duration += (*it);
    time_alloc.push_back(*it);
  }
  Eigen::MatrixX3d cpts;
  cpts.resize(R, 3);
  for (int i = 0; i < R; ++i) {
    cpts(i, 0) = msg->cpts[i].x;
    cpts(i, 1) = msg->cpts[i].y;
    cpts(i, 2) = msg->cpts[i].z;
  }

  Bernstein::Bezier bezier_traj;
  bezier_traj.setOrder(N);
  bezier_traj.setTime(time_alloc);
  bezier_traj.setControlPoints(cpts);

  // ROS_DEBUG("Bezier trajectory received, id: %d, order: %d, duration: %f", id, N, duration);

  trajectory trajs;
  trajs.resize(duration * 20 + 1);
  for (int i = 0; i < duration * 20; ++i) {
    double t     = i * 0.05;
    trajs[i].pos = bezier_traj.getPos(t);
    trajs[i].vel = bezier_traj.getVel(t);
    trajs[i].acc = bezier_traj.getAcc(t);
    std::cout << t << " " << trajs[i].pos.transpose() << std::endl;
  }

  // ROS_DEBUG("Bezier trajectory converted to trajectory, id: %d, order: %d, duration: %f", id, N,
  //          duration);

  visualization_msgs::MarkerArray marker_array_traj;
  marker_array_traj = trajectory2ColoredMarkerArray(trajs, MAX_VEL, 1, "drone" + std::to_string(id),
                                                    SCALE, "vel", AGENT_ID, N_AGENTS);
  // ROS_DEBUG("Bezier trajectory converted to marker array, id: %d, order: %d, duration: %f", id,
  // N, duration);
  colored_traj_pub_.publish(marker_array_traj);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "color_traj_node");
  ros::NodeHandle nh("~");
  nh.param("agent_id", AGENT_ID, 0);
  nh.param("n_agents", N_AGENTS, 1);
  nh.param("max_vel", MAX_VEL, 3.0);
  nh.param("scale", SCALE, 0.1);

  ros::Subscriber sub_odom = nh.subscribe("bezier_traj", 100, BezierCallback);
  colored_traj_pub_        = nh.advertise<visualization_msgs::MarkerArray>("color_traj", 100);
  ros::spin();
}
