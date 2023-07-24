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

// #define FAKE_PERCEPTION 0  // 1: use fake perception; 0: use real perception

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
// #include <path_searching/fake_risk_hybrid_a_star.h>
#include <path_searching/risk_hybrid_a_star.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <plan_env/fake_dsp_map.h>
#include <plan_env/risk_voxel.h>
#include <sensor_msgs/PointCloud2.h>
#include <traj_utils/BezierTraj.h>
#include <bernstein/bezier_optimizer.hpp>
// #include <plan_manager/mader_deconfliction.hpp>
#include <sfc_gen/firi.hpp>
#include <traj_coordinator/mader.hpp>
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
  // bool is_odom_local      = false;
  // bool is_fake_perception = false;

  double min_volumn = 0.5;
  // double max_vel        = 3.0;
  // double max_acc        = 6.0;
  double opt_max_vel = 3.0;
  double opt_max_acc = 4.0;
  // double delta_corridor = 0.3;
  double init_range = 1.0;

  // bool  use_height_limit = true;
  // float height_limit_max = 2.2f;
  // float height_limit_min = 0.f;
  // bool  sample_z_acc     = true;

  // float  a_star_acc_sample_step  = 2.f;
  double corridor_tau = 0.4f; /* time span for corridor generation */
  // float  a_star_search_time_step = 0.4f;
  // float  expand_safety_distance  = 0.2f;

  // float risk_threshold_motion_primitive = 0.15;
  // float risk_threshold_single_voxel     = 0.15;
  // float risk_threshold_corridor         = 2.5;
  // float risk_threshold_replan           = 20;

  // int trajectory_piece_max_size = 12;
  // int nmpc_receive_points_num = 20;
  // double planning_time_step        = 0.05;
  // bool   is_rviz_map_center_locked = false;

  // double waypoint_distance = 4.0;
  // double goal_reach_threshold = 1.0;
  // double replan_time_threshold = 0.5;

  /* New */
  BaselineParameters(const ros::NodeHandle &nh) {
    // nh.getParam("init_x", init_x);
    // nh.getParam("init_y", init_y);
    // nh.getParam("init_z", init_z);
    // nh.getParam("init_qx", init_qx);
    // nh.getParam("init_qy", init_qy);
    // nh.getParam("init_qz", init_qz);
    // nh.getParam("init_qw", init_qw);
    // nh.getParam("is_odom_local", is_odom_local);
    // nh.getParam("is_fake_perception", is_fake_perception);
    nh.getParam("planner/corridor_tau", corridor_tau);

    nh.getParam("corridor/init_range", init_range);
    nh.getParam("corridor/min_volumn", min_volumn);

    nh.getParam("optimizer/max_vel_optimization", opt_max_vel);
    nh.getParam("optimizer/max_acc_optimization", opt_max_acc);
    // nh.getParam("optimizer/delta_corridor", delta_corridor);

    // nh.getParam("astar/use_height_limit", use_height_limit);
    // nh.getParam("astar/height_limit_max", height_limit_max);
    // nh.getParam("astar/height_limit_min", height_limit_min);
    // nh.getParam("astar/sample_z_acc", sample_z_acc);
    // nh.getParam("astar/a_star_acc_sample_step", a_star_acc_sample_step);
    // nh.getParam("astar/a_star_search_time_step", a_star_search_time_step);
    // nh.getParam("astar/risk_threshold_motion_primitive", risk_threshold_motion_primitive);
    // nh.getParam("astar/expand_safety_distance", expand_safety_distance);
    // nh.getParam("astar/nmpc_receive_points_num", nmpc_receive_points_num);
    //
    // nh.getParam("corridor/risk_threshold_single_voxel", risk_threshold_single_voxel);
    // nh.getParam("corridor/risk_threshold_corridor", risk_threshold_corridor);
    //
    // nh.getParam("rviz_map_center_locked", is_rviz_map_center_locked);
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
  bool replan(double                 t,
              const Eigen::Vector3d &start_pos,
              const Eigen::Vector3d &start_vel,
              const Eigen::Vector3d &start_acc,
              const Eigen::Vector3d &goal_pos);
  void setStartTime(double t) { prev_traj_start_time_ = t; }

  void showAstarPath();

  Bernstein::Bezier getTrajectory() const { return traj_; }

  inline Eigen::Vector3d getPos(double t) const;
  inline Eigen::Vector3d getVel(double t) const;
  inline Eigen::Vector3d getAcc(double t) const;

  /* Visualization */
  visualizer::Visualizer::Ptr visualizer_;

  typedef std::shared_ptr<BaselinePlanner> Ptr;

 private:
  /* Helper function */
  Eigen::Matrix<double, 6, 4> getInitCorridor(const Eigen::Vector3d &left_higher_corner,
                                              const Eigen::Vector3d &right_lower_corner);

  void showObstaclePoints(const std::vector<Eigen::Vector3d> &points);
  void addAgentsTrajectoryToMap();
  void setEmptyTrajectory(const Eigen::Vector3d &pos);

 private:
  /* ROS */
  ros::NodeHandle nh_;
  ros::Publisher  obstacle_pub_;

  BaselineParameters cfg_;

  /* Shared Pointers */
  RiskVoxel::Ptr           map_;
  RiskHybridAstar::Ptr     a_star_;
  traj_opt::BezierOpt::Ptr traj_optimizer_;    /** Trajectory optimizer */
  MADER::Ptr               collision_avoider_; /* multi-agent collision avoidance policy*/

  /* Trajectory */
  int    traj_idx_;        /** Trajectory index */
  double traj_start_time_; /** current trajectory start time */
  double prev_traj_start_time_;

  Bernstein::Bezier traj_; /** Trajectory class */
};

inline Eigen::Vector3d BaselinePlanner::getPos(double t) const {
  return traj_.getPos(t - prev_traj_start_time_);
}

inline Eigen::Vector3d BaselinePlanner::getVel(double t) const {
  return traj_.getVel(t - prev_traj_start_time_);
}

inline Eigen::Vector3d BaselinePlanner::getAcc(double t) const {
  return traj_.getAcc(t - prev_traj_start_time_);
}

#endif  // _BASELINE_H_
