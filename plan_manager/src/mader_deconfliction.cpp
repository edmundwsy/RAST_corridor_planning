/**
 * @file mader_deconfliction.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <plan_manager/mader_deconfliction.hpp>

void MADER::init() {
  separator_solver_ = new separator::Separator();

  swarm_sub_ = nh_.subscribe("/broadcast_traj", 1, &MADER::trajectoryCallback, this);
  nh_.param("drone_id", drone_id_, 0);
  nh_.param("swarm/num_robots", num_robots_, 3);  // TODO(1): dynamic reconfigure

  is_checking_                         = false;
  have_received_traj_while_checking_   = false;
  have_received_traj_while_optimizing_ = false;

  swarm_trajs_.resize(num_robots_ - 1);
  traj_id_to_index_.clear();

  /* Assign traj_id to k in swarm_trajs_ */
  int j = 0;
  for (int i = 0; i < num_robots_ - 1; i++) {
    if (i != drone_id_) {
      traj_id_to_index_[i] = j;
    }
    j++;
  }
}

/**
 * @brief Save boardcasted trajectory to swarm_trajs_
 * @param traj_msg 
 */
void MADER::trajectoryCallback(const traj_utils::BezierTraj::ConstPtr &traj_msg) {
  if (is_checking_) {
    have_received_traj_while_checking_ = true;
  } else {
    have_received_traj_while_optimizing_ = false;
  }
  int id = traj_msg->drone_id;

  /* filter out ego trajectories */
  if (id == drone_id_) {
    return;
  }

  int k = traj_id_to_index_[id];

  /* remove old trajectories from swarm_trajs_ buffer */
  ros::Time now = ros::Time::now();
  while (!swarm_trajs_[k].empty()) {
    /* If trajectory ends earlier */
    if (ros::Time::now() > swarm_trajs_[k].front().time_end) {
      swarm_trajs_[k].pop();
    } else {
      break;
    }
  }
  ROS_INFO("Traj size after remove: %d", (int)swarm_trajs_[k].size());

  /* update swarm_trajs_ */
  int n_seg = traj_msg->duration.size();
  int order = traj_msg->order + 1;

  SwarmTraj traj;
  traj.id            = traj_msg->drone_id;
  traj.time_received = ros::Time::now();
  traj.time_start    = traj_msg->start_time;

  ROS_INFO("Agent %i traj start time: %f", traj.id, traj.time_start.toSec());
  ROS_INFO("Agent %i traj received time: %f", traj.id, traj.time_received.toSec());

  for (int i = 0; i < n_seg; i++) {
    traj.duration   = traj_msg->duration[i];
    traj.time_start = traj_msg->start_time + ros::Duration(traj.duration) * i;
    traj.time_end   = traj.time_start + ros::Duration(traj.duration);
    traj.control_points.resize(order, 3);
    for (int j = 0; j < order; j++) {
      int k = j + order * i;
      traj.control_points.row(j) << traj_msg->cpts[k].x, traj_msg->cpts[k].y, traj_msg->cpts[k].z;
    }
    swarm_trajs_[k].push(traj);
  }

  /* print intermediate results */
  ROS_INFO("Drone %d has %d trajectories for drone %d", drone_id_, (int)swarm_trajs_[k].size(), k);
}

/**
 * @brief obtains control points of other drones in the prediction horizon
 *
 * @param pts input obstacle points buffer
 * @param t prediction horizon
 */
void MADER::getObstaclePoints(std::vector<Eigen::Vector3d> &pts, double horizon) {
  ros::Time t_start = ros::Time::now();
  ros::Time t_end   = t_start + ros::Duration(horizon);
  for (int i = 0; i < num_robots_ - 1; i++) { /* iterate all robots in the buffer */
    SwarmTraj traj;
    while (!swarm_trajs_[i].empty()) {
      traj = swarm_trajs_[i].front();

      /* If trajectory ends earlier */
      if (t_start > traj.time_end) {
        swarm_trajs_[i].pop();
        continue;
      }

      if (t_end > traj.time_start || t_start < traj.time_end) {
        ROS_INFO("Drone %d : pushing %d obstacle points", traj.id, (int)traj.control_points.rows());
        for (int j = 0; j < traj.control_points.rows(); j++) {
          pts.push_back(traj.control_points.row(j));
        }
      }
      break;
    }
  }
}

/**
 * @brief check if the trajectory is collision free
 *
 * @param traj_msg
 * @return true    safe
 * @return false   unsafe
 */
bool MADER::isSafeAfterOpt(const Bernstein::Bezier &traj) {
  is_checking_ = true;
  /* collision checking starts */

  double          t = traj.getDuration();
  Eigen::MatrixXd cpts;
  traj.getCtrlPoints(cpts);
  /* checkin: check waypoints */

  std::vector<Eigen::Vector3d> pointsA;
  std::vector<Eigen::Vector3d> pointsB;

  for (int i = 0; i < cpts.rows(); i++) {
    pointsA.push_back(cpts.row(i));
  }

  Eigen::Vector3d n_k;
  double          d_k;

  for (int k = 0; k < swarm_trajs_.size(); k++) {
    if (swarm_trajs_[k].empty()) {
      continue;
    }
    Eigen::MatrixXd cpts = swarm_trajs_[k].front().control_points;
    for (int i = 0; i < cpts.rows(); i++) {
      pointsB.push_back(cpts.row(i));
    }

    if (!separator_solver_->solveModel(n_k, d_k, pointsA, pointsB)) {
      ROS_WARN("Drone %d will collides with drone %d", drone_id_, k);
      is_checking_ = false;
      return false;
    }
  }

  is_checking_ = false;
  return true;
}
