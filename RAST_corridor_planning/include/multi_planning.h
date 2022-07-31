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
#include <vector>

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

  int trajectory_piece_max_size = 12;
  int nmpc_receive_points_num   = 20;

  double planning_time_step           = 0.05;
  double max_differentiated_current_a = 4.0;
  bool   is_rviz_map_center_locked    = false;

  std::vector<double> factors;
  double              factor_pos;
  double              factor_vel;
  double              factor_acc;
  double              factor_jrk;
  double              factor_snp;

  double goal_x = 60.0;
  double goal_y = 0.0;
  double goal_z = 1.5;

  PlannerConfig(const ros::NodeHandle &nh) {
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

    nh.getParam("/planning_node/delta_corridor", delta_corridor);
    nh.getParam("/planning_node/pos_factor", factor_pos);
    nh.getParam("/planning_node/vel_factor", factor_vel);
    nh.getParam("/planning_node/acc_factor", factor_acc);
    nh.getParam("/planning_node/jerk_factor", factor_jrk);
    nh.getParam("/planning_node/snap_factor", factor_snp);
    factors = {factor_pos, factor_vel, factor_acc, factor_jrk, factor_snp};

    nh.getParam("/planning_node/planning_time_step", planning_time_step);
    nh.getParam("/planning_node/a_star_acc_sample_step", a_star_acc_sample_step);
    nh.getParam("/planning_node/a_star_search_time_step", a_star_search_time_step);

    nh.getParam("/planning_node/risk_threshold_motion_primitive", risk_threshold_motion_primitive);
    nh.getParam("/planning_node/risk_threshold_single_voxel", risk_threshold_single_voxel);
    nh.getParam("/planning_node/risk_threshold_corridor", risk_threshold_corridor);

    nh.getParam("/planning_node/p_goal_x", goal_x);
    nh.getParam("/planning_node/p_goal_y", goal_y);
    nh.getParam("/planning_node/p_goal_z", goal_z);

    nh.getParam("/planning_node/rviz_map_center_locked", is_rviz_map_center_locked);
  }
};

class Planner {
 private:
  /********** TRAJECTORY PLANNING **********/
  traj_opt::CorridorMiniSnap::Ptr _traj_optimizer; /** Trajectory optimizer */
  traj_opt::Trajectory            _traj;           /** Trajectory */

  Astar _astar_planner; /** A* path finding */

  Eigen::Vector3d    _odom_pos; /** quadrotor's current position */
  Eigen::Vector3d    _odom_vel; /** quadrotor's current velocity */
  Eigen::Vector3d    _odom_acc; /** quadrotor's current acceleration */
  Eigen::Quaternionf _odom_att; /** quadrotor's current attitude as a quaternion */

  double _prev_t, _prev_vx, _prev_vy, _prev_vz;
  double _prev_traj_end_time;  /// previous trajectory end time

  /********** MAP **********/
  int   _map_x_limit, _map_y_limit, _map_z_limit; /** Map limits */
  int   _map_size;
  float _map_half_length, _map_half_width, _map_half_height;

  /********** ROS UTILS **********/
  ros::NodeHandle _nh;
  ros::Timer      _traj_timer;
  ros::Subscriber _future_risk_sub, _pose_sub, _vel_sub;
  ros::Publisher  _traj_pub, _corridor_pub;

  /********** CONFIG **********/
  PlannerConfig _config;
  float         _ref_direction_angle;

  /********** DATA **********/
  geometry_msgs::PoseStamped _map_center;  // TODO(@siyuan): change this to Eigen::Vector3d
  ros::Time                  _traj_start_time;
  ros::Time                  _traj_end_time;

  double _traj_planning_start_time;

  /** @brief Risk map in map frame. usage: [spatial_index][temporal_index] */
  float _future_risk[VOXEL_NUM][RISK_MAP_NUMBER];

  Eigen::Vector3d _p_store_for_em, _v_store_for_em, _a_store_for_em;

  /********** BOOLEANS **********/
  bool _is_future_risk_updated;
  bool _is_future_risk_locked;
  bool _is_safety_mode_enabled;
  bool _is_odom_received;
  bool _is_trajectory_initialized;
  bool _is_rviz_center_locked;
  bool _is_state_locked;

  /********** VISUALIZATIONS **********/
  Visualizer::Ptr _vis;

 public:
  Planner(ros::NodeHandle &nh, const PlannerConfig &conf);
  ~Planner() {}

  int getPointSpatialIndexInMap(const Eigen::Vector3d &p, const Eigen::Vector3d &c);

  void FutureRiskCallback(const std_msgs::Float32MultiArrayConstPtr &risk_msg);
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void VelCallback(const geometry_msgs::TwistStamped &msg);
  void TrajTimerCallback(const ros::TimerEvent &event);
  bool OptimizationInCorridors(const decomp_ros_msgs::DynPolyhedronArray msg,
                               const Eigen::Vector3d                     start_pos);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::unique_ptr<Planner> Ptr;
};

}  // namespace planner

#endif  // _MULTI_PLANNING_H_