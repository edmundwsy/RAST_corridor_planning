/**
 * @file mader_deconfliction.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef MADER_DECONFLICTION_H
#define MADER_DECONFLICTION_H

#include <ros/ros.h>
#include <traj_utils/BezierTraj.h>
#include <Eigen/Eigen>
#include <map>
#include <memory>
#include <queue>
#include <traj_utils/bernstein.hpp>
#include <vector>
#include "separator.hpp"

struct SwarmTraj {
  int              id;
  ros::Time        time_received;
  ros::Time        time_start;
  ros::Time        time_end;
  double           duration;
  Eigen::MatrixX3d control_points;
};

class MADER {
 public:
  /* constructor */
  MADER(ros::NodeHandle &nh) : nh_(nh) {}
  ~MADER() {}

  /* functions */
  void init();
  void reset();
  void trajectoryCallback(const traj_utils::BezierTraj::ConstPtr &traj_msg);
  bool isSafeAfterOpt(const Bernstein::Bezier &traj);
  bool isSafeAfterChk() { return !have_received_traj_while_checking_; }
  bool updateTrajObstacles(/* arguments */);
  void getObstaclePoints(std::vector<Eigen::Vector3d> &pts, double t);

  typedef std::shared_ptr<MADER> Ptr;

 private:
  ros::NodeHandle nh_;
  ros::Subscriber swarm_sub_;
  /* variables */
  std::vector<std::queue<SwarmTraj>> swarm_trajs_;
  std::map<int, int>                 traj_id_to_index_;

  bool is_planner_initialized_;
  bool have_received_traj_while_checking_;
  bool have_received_traj_while_optimizing_;
  bool is_traj_safe_;
  bool is_checking_;

  int drone_id_;
  int num_robots_;

  separator::Separator *separator_solver_;
};

#endif  // MADER_DECONFLICTION_H