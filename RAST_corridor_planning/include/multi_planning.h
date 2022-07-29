/**
 * @file multi_planning.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-07-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _MULTI_PLANNING_H_
#define _MULTI_PLANNING_H_

#include <CorridorMiniSnap/corridor_minisnap.h>
#include <geometry_msgs/TwistStamped.h>
#include <traj_utils/PolyTraj.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>
#include <ctime>
#include <queue>

#include "decomp_ros_msgs/DynPolyhedronArray.h"
#include "decomp_utils.hpp"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "mav_msgs/default_topics.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "risk_aware_kinodynamic_a_star.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"
#include "visualizer.hpp"

namespace planner {

struct PlannerConfig {
  /* data */
  double max_vel              = 3.0;
  double max_acc              = 4.0;
  double max_vel_optimization = 3.0;
  double max_acc_optimization = 4.0;
  double delta_corridor       = 0.3;

  bool  use_height_limit = true;
  float height_limit_max = 2.2f;
  float height_limit_min = 0.f;
  bool  sample_z_acc     = true;

  float a_star_acc_sample_step  = 2.f;
  float a_star_search_time_step = 0.4f;
  float expand_safety_distance  = 0.2f;

  float risk_threshold_motion_primitive = 0.15;
  float risk_threshold_single_voxel     = 0.15;
  float risk_threshold_corridor         = 2.5;

  int    trajectory_piece_max_size = 12;
  int    nmpc_receive_points_num   = 20;
  double planning_time_step        = 0.05;

  double max_differentiated_current_a = 4.0;

  double goal_x = 60.0, goal_y = 0.0, goal_z = 1.5;

  PlannerConfig(const ros::NodeHandle &nh) {
    nh.getParam("/planning_node/p_goal_x", goal_x);
    nh.getParam("/planning_node/p_goal_y", goal_y);
    nh.getParam("/planning_node/p_goal_z", goal_z);
    nh.getParam("/planning_node/max_vel", max_vel);
    nh.getParam("/planning_node/max_acc", max_acc);

    nh.getParam("/planning_node/max_vel_optimization", max_vel_optimization);
    nh.getParam("/planning_node/max_acc_optimization", max_acc_optimization);
    nh.getParam("/planning_node/max_differentiated_current_a", max_differentiated_current_a);

    nh.getParam("/planning_node/use_height_limit", use_height_limit);
    nh.getParam("/planning_node/height_limit_max", height_limit_max);
    nh.getParam("/planning_node/height_limit_min", height_limit_min);
    nh.getParam("/planning_node/sample_z_acc", sample_z_acc);
    nh.getParam("/planning_node/expand_safety_distance", expand_safety_distance);
    nh.getParam("/planning_node/trajectory_piece_max_size", trajectory_piece_max_size);
    nh.getParam("/planning_node/nmpc_receive_points_num", nmpc_receive_points_num);

    nh.getParam("/planning_node/pos_factor", factors[0]);
    nh.getParam("/planning_node/vel_factor", factors[1]);
    nh.getParam("/planning_node/acc_factor", factors[2]);
    nh.getParam("/planning_node/jerk_factor", factors[3]);
    nh.getParam("/planning_node/snap_factor", factors[4]);
    nh.getParam("/planning_node/delta_corridor", delta_corridor);

    nh.getParam("/planning_node/planning_time_step", planning_time_step);
    nh.getParam("/planning_node/a_star_acc_sample_step", a_star_acc_sample_step);
    nh.getParam("/planning_node/a_star_search_time_step", a_star_search_time_step);

    nh.getParam("/planning_node/rviz_map_center_locked", rviz_map_center_locked);

    nh.getParam("/planning_node/risk_threshold_motion_primitive", risk_threshold_motion_primitive);
    nh.getParam("/planning_node/risk_threshold_single_voxel", risk_threshold_single_voxel);
    nh.getParam("/planning_node/risk_threshold_corridor", risk_threshold_corridor);
  }
};

class Planner {
 private:
  /********** TRAJECTORY PLANNING **********/
  traj_opt::CorridorMiniSnap::Ptr _traj_optimizer;
  traj_opt::Trajectory            _traj;

  Astar _aster_planner;

  Eigen::Vector3d    _pos; /** quadrotor's current position */
  Eigen::Vector3d    _vel; /** quadrotor's current velocity */
  Eigen::Vector3d    _acc; /** quadrotor's current acceleration */
  Eigen::Quaternionf _att; /** quadrotor's current attitude as a quaternion */

  /********** ROS UTILS **********/
  ros::NodeHandle _nh;
  ros::Timer      _traj_timer;
  ros::Subscriber _future_risk_sub, _pose_sub, _vel_sub;
  ros::Publisher  _traj_pub;

  /********** CONFIG **********/
  PlannerConfig _config;

  /********** DATA **********/
  geometry_msgs::PoseStamped _pose;

  float _future_risk[VOXEL_NUM][RISK_MAP_NUMBER];  /** future risk map, global frame **/

  /********** BOOLEANS **********/
  _is_future_risk_updated = false;
  _is_future_risk_locked = false;
  _is_safety_mode_enabled = false;
  _is_position_received = false;
  _is_trajectory_initialized = false;
  _is_rviz_center_locked = false;
  _is_state_locked = false;
  

 public:
  Planner(const PlannerConfig conf, ros::NodeHandle &nh);
  ~Planner();

  FutureRiskCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
  PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  VelCallback(const geometry_msgs::TwistStamped& msg);
  TrajTimerCallback(const ros::TimerEvent &event);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::unique_ptr<Planner> Ptr;
}

}  // namespace planner

#endif  // _MULTI_PLANNING_H_