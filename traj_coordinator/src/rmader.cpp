/**
 * @file rmader.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-12-27
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <traj_coordinator/rmader.hpp>

void RMADER::init() {
  separator_solver_ = new separator::Separator();

  swarm_sub_ =
      nh_.subscribe("/broadcast_traj", 1, &RMADER::trajectoryCallback, dynamic_cast<MADER *>(this));
  nh_.param("drone_id", drone_id_, 0);
  nh_.param("swarm/num_robots", num_robots_, 3);
  nh_.param("swarm/drone_size_x", drone_size_x_, 0.3);
  nh_.param("swarm/drone_size_y", drone_size_y_, 0.3);
  nh_.param("swarm/drone_size_z", drone_size_z_, 0.4);     // avoid z-axis wind effect
  nh_.param("swarm/delay_check_length", delta_dc_, 0.05);  // 50ms by default

  ego_size_ = Eigen::Vector3d(drone_size_x_, drone_size_y_, drone_size_z_);

  /* Initialize Booleans */
  is_checking_                         = false;
  have_received_traj_while_checking_   = false;
  have_received_traj_while_optimizing_ = false;

  swarm_trajs_.resize(num_robots_ - 1);
  drone_id_to_index_.clear();
  index_to_drone_id_.clear();

  /* Assign traj_id to k in swarm_trajs_ */
  int j = 0;
  for (int i = 0; i < num_robots_; i++) {
    if (i != drone_id_) {
      drone_id_to_index_[i] = j;
      index_to_drone_id_[j] = i;
      j++;
    }
  }

  /* Pre-allocate Ego Cube */
  ego_cube_.col(0) = Eigen::Vector3d(drone_size_x_ / 2, drone_size_y_ / 2, drone_size_z_ / 2);
  ego_cube_.col(1) = Eigen::Vector3d(drone_size_x_ / 2, -drone_size_y_ / 2, drone_size_z_ / 2);
  ego_cube_.col(2) = Eigen::Vector3d(drone_size_x_ / 2, drone_size_y_ / 2, -drone_size_z_ / 2);
  ego_cube_.col(3) = Eigen::Vector3d(drone_size_x_ / 2, -drone_size_y_ / 2, -drone_size_z_ / 2);
  ego_cube_.col(4) = Eigen::Vector3d(-drone_size_x_ / 2, drone_size_y_ / 2, drone_size_z_ / 2);
  ego_cube_.col(5) = Eigen::Vector3d(-drone_size_x_ / 2, -drone_size_y_ / 2, drone_size_z_ / 2);
  ego_cube_.col(6) = Eigen::Vector3d(-drone_size_x_ / 2, drone_size_y_ / 2, -drone_size_z_ / 2);
  ego_cube_.col(7) = Eigen::Vector3d(-drone_size_x_ / 2, -drone_size_y_ / 2, -drone_size_z_ / 2);
}

/**
 * @brief check if the trajectory is collision free
 *  check if two sets of convex hulls linear separable
 */
bool RMADER::collisionCheck(const Bernstein::Bezier &traj) {
  Eigen::MatrixXd cpts;
  traj.getCtrlPoints(cpts);
  /* checkin: check waypoints */

  std::vector<Eigen::Vector3d> pointsA;
  std::vector<Eigen::Vector3d> pointsB;

  /* Push ego trajectory convex hull to buffer */
  loadVertices(pointsA, cpts);

  /* checkin: check other trajectories */
  for (int k = 0; k < swarm_trajs_.size(); k++) { /* iterate all robots in the buffer */
    if (!swarm_trajs_[k].empty()) {               /* if the buffer is not empty */
      while (swarm_trajs_[k].front().time_end < ros::Time::now()) {
        swarm_trajs_[k].pop();
        if (swarm_trajs_[k].empty()) {
          break;
        }
      }
    }

    if (!swarm_trajs_[k].empty()) {
      Eigen::Vector3d n_k;
      double          d_k;
      pointsB.clear();
      Eigen::MatrixXd cpts = swarm_trajs_[k].front().control_points;
      loadVertices(pointsB, cpts);
      ROS_INFO("[CA|A%i] time span (%f, %f)", index_to_drone_id_[k],
               swarm_trajs_[k].front().time_start.toSec() - ros::Time::now().toSec(),
               swarm_trajs_[k].front().time_end.toSec() - ros::Time::now().toSec());

      if (!separator_solver_->solveModel(n_k, d_k, pointsA, pointsB)) {
        ROS_INFO("[CA|A%i] Cannot find linear separation plane", index_to_drone_id_[k]);
        ROS_WARN("[CA] Drone %i will collides with drone %i", drone_id_, index_to_drone_id_[k]);
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief check if the trajectory is collision free
 *  check if two sets of convex hulls linear separable
 *
 * @param traj_msg
 * @return true    safe
 * @return false   unsafe
 */
bool RMADER::isSafeAfterOpt(const Bernstein::Bezier &traj) {
  is_checking_ = true;
  /* collision checking starts */
  bool is_safe = collisionCheck(traj);
  is_checking_ = false;
  return is_safe;
}

bool RMADER::isSafeAfterDelayCheck(const Bernstein::Bezier &traj) {
  double t        = 0.0;
  double dc_start = ros::Time::now().toSec();

  while (t < delta_dc_) {
    t            = ros::Time::now().toSec() - dc_start;
    bool is_safe = collisionCheck(traj);
    if (!is_safe) {
      return false;
    }
  }
  ROS_WARN("[CA] Drone %i passed the delay check", drone_id_);
  return true;

  /* TODO: delay check every 2~5ms */
  // TODO(1.9): modify the plan_manager to achieve delay check.
  // TODO(1.9): a new thread is required. ? not necessary?
}

// /**
//  * @brief input vertices of trajectory convex hull, get Minkowski sum of the convex
//  * hull and ego polytope, and push these vertices into the buffer `pts`
//  * @param pts : points buffer to be filled
//  * @param cpts: control points (vertices of trajectory convex hull)
//  */
// void RMADER::loadVertices(std::vector<Eigen::Vector3d> &pts, Eigen::MatrixXd &cpts) {
//   for (int i = 0; i < cpts.rows(); i++) {
//     /* Add trajectory control point */
//     Eigen::Vector3d pt = cpts.row(i);
//     for (int j = 0; j < 8; j++) {
//       pts.push_back(pt + ego_cube_.col(j));
//     }
//   }
// }

// /**
//  * @brief DEPRECATED
//  * @param points  trajectory waypoints to be filled
//  * @param i : index of the agent
//  * @param tf
//  */
// void RMADER::getAgentsTrajectory(std::vector<Eigen::Vector3d> &points, int idx_agent, double dt)
// {
//   ros::Time t0 = ros::Time::now() + ros::Duration(dt);

//   std::queue<SwarmTraj> traj_queue = swarm_trajs_[idx_agent];
//   while (!traj_queue.empty()) {
//     Eigen::MatrixXd cpts = traj_queue.front().control_points;

//     if (traj_queue.front().time_start < t0 && traj_queue.front().time_end > t0) {
//       /* Adding trajectory points to the buffer */
//       /* Here we use control points as an approximation */
//       for (int j = 0; j < 2; j++) {
//         /* control point number: 0 2 */
//         Eigen::Vector3d pt = cpts.row(2 * j);
//         points.push_back(pt);
//         ROS_INFO_STREAM("[CA|A" << index_to_drone_id_[idx_agent] << "] got trajectory at "
//                                 << t0.toSec() - traj_queue.front().time_start.toSec()
//                                 << " | pt: " << pt.transpose());
//         t0 += ros::Duration(dt);
//         if (t0 > traj_queue.front().time_end) {
//           break;
//         }
//       }
//     } else {
//       traj_queue.pop();
//       continue;
//     }
//   }
// }

// /**
//  * @brief get waypoints of the agent 'idx_agenq' at time 't'
//  * @param pts
//  * @param idx_agent
//  * @param t
//  */
// void RMADER::getWaypoints(std::vector<Eigen::Vector3d> &pts, int idx_agent, ros::Time t) {
//   std::queue<SwarmTraj> traj_queue = swarm_trajs_[idx_agent];
//   while (!traj_queue.empty()) {
//     if (traj_queue.front().time_start < t && traj_queue.front().time_end > t) {
//       double dt = (t - traj_queue.front().time_start).toSec();
//       pts.push_back(traj_queue.front().piece.getPos(dt));
//       ROS_INFO_STREAM("[CA|A" << index_to_drone_id_[idx_agent] << "] got trajectory at " << dt
//                               << " | wpt: " << pts.back().transpose());
//       break;
//     } else {
//       traj_queue.pop();
//     }
//   }
// }
