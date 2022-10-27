/**
 * @file baseline.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-24
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _BASELINE_H_
#define _BASELINE_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <path_searching/risk_hybrid_a_star.h>
#include <plan_env/risk_voxel.h>
#include <traj_utils/BezierTraj.h>
#include <bernstein/bezier_optimizer.hpp>
#include <traj_utils/bernstein.hpp>
#include <traj_utils/corridor.hpp>
#include <traj_utils/visualizer.hpp>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <string>
#include <termcolor.hpp>  // for colored output
#include <vector>

struct BaselineParameters {
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
  float risk_threshold_replan           = 20;

  int trajectory_piece_max_size = 12;
  int nmpc_receive_points_num   = 20;

  double planning_time_step           = 0.05;
  double max_differentiated_current_a = 4.0;
  bool   is_rviz_map_center_locked    = false;

  double goal_x                = 60.0;
  double goal_y                = 0.0;
  double goal_z                = 1.5;
  double waypoint_distance     = 4.0;
  double goal_reach_threshold  = 1.0;
  double replan_time_threshold = 0.5;

  BaselineParameters(const ros::NodeHandle &nh) {
    nh.getParam("planner/p_goal_x", goal_x);
    nh.getParam("planner/p_goal_y", goal_y);
    nh.getParam("planner/p_goal_z", goal_z);
    nh.getParam("planner/max_vel", max_vel);
    nh.getParam("planner/max_acc", max_acc);
    nh.getParam("planner/goal_reach_threshold", goal_reach_threshold);
    nh.getParam("planner/risk_threshold_replan", risk_threshold_replan);
    nh.getParam("planner/replan_time_threshold", replan_time_threshold);
    nh.getParam("planner/max_differentiated_current_a", max_differentiated_current_a);
    nh.getParam("planner/planning_time_step", planning_time_step);
    nh.getParam("planner/trajectory_piece_max_size", trajectory_piece_max_size);

    nh.getParam("optimizer/max_vel_optimization", max_vel_optimization);
    nh.getParam("optimizer/max_acc_optimization", max_acc_optimization);
    nh.getParam("optimizer/delta_corridor", delta_corridor);

    nh.getParam("astar/use_height_limit", use_height_limit);
    nh.getParam("astar/height_limit_max", height_limit_max);
    nh.getParam("astar/height_limit_min", height_limit_min);
    nh.getParam("astar/sample_z_acc", sample_z_acc);
    nh.getParam("astar/a_star_acc_sample_step", a_star_acc_sample_step);
    nh.getParam("astar/a_star_search_time_step", a_star_search_time_step);
    nh.getParam("astar/risk_threshold_motion_primitive", risk_threshold_motion_primitive);
    nh.getParam("astar/expand_safety_distance", expand_safety_distance);
    nh.getParam("astar/nmpc_receive_points_num", nmpc_receive_points_num);

    nh.getParam("corridor/risk_threshold_single_voxel", risk_threshold_single_voxel);
    nh.getParam("corridor/risk_threshold_corridor", risk_threshold_corridor);

    nh.getParam("waypoint_distance", waypoint_distance);
    nh.getParam("rviz_map_center_locked", is_rviz_map_center_locked);
  }
};

class BaselinePlanner {
 public:
  BaselinePlanner(ros::NodeHandle &nh, const BaselineParameters &params) : nh_(nh), cfg_(params) {}
  ~BaselinePlanner() {}

  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void clickCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  void init();
  bool plan();

  void showAstarPath();

  typedef std::shared_ptr<BaselinePlanner> Ptr;

 private:
  /* ROS */
  ros::NodeHandle    nh_;
  ros::Subscriber    click_sub_, pose_sub_;
  BaselineParameters cfg_;

  Eigen::Vector3d    odom_pos_; /** quadrotor's current position */
  Eigen::Vector3d    odom_vel_; /** quadrotor's current velocity */
  Eigen::Vector3d    odom_acc_; /** quadrotor's current acceleration */
  Eigen::Quaterniond odom_att_; /** quadrotor's current attitude as a quaternion */

  double prev_pt_, prev_px_, prev_py_, prev_pz_; /** previous point */
  double prev_vt_, prev_vx_, prev_vy_, prev_vz_; /** previous velocity */
  double prev_opt_end_time_;                     /** previous trajectory end time */

  /* Shared Pointers */
  RiskVoxel::Ptr           map_;
  RiskHybridAstar::Ptr     a_star_;
  traj_opt::BezierOpt::Ptr traj_optimizer_; /** Trajectory optimizer */

  /* Trajectory */
  Bernstein::Bezier traj_;     /** Trajectory */
  int               traj_idx_; /** Trajectory index */

  /* Waypoints */

  /* Booleans */
  bool is_state_locked_;      /** State lock */
  bool is_velocity_received_; /** Velocity received */
  bool is_odom_received_;     /** Odom received */

  /* Visualization */
  visualizer::Visualizer::Ptr visualizer_;
};

#endif  // _BASELINE_H_