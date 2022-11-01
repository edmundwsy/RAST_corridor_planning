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
  swarm_sub_   = nh_.subscribe("/boardcast_traj", 1, &MADER::trajectoryCallback, this);
  is_checking_ = false;
  have_received_traj_while_checking_   = false;
  have_received_traj_while_optimizing_ = false;
}

void MADER::trajectoryCallback(const traj_utils::BezierTraj::ConstPtr &traj_msg) {
  if (is_checking_) {
    have_received_traj_while_checking_ = true;
  } else {
    have_received_traj_while_optimizing_ = false;
  }
}

void MADER::getObstaclePoints(std::vector<Eigen::Vector3d> &pts) {}


bool MADER::safetyCheckAfterOpt(const Bernstein::Bezier &traj) {
  is_checking_ = true;
  Eigen::MatrixXd cpts;
  traj.getCtrlPoints(cpts);
  /* checkin: check waypoints */
  is_checking_ = false;
  return true;
}
