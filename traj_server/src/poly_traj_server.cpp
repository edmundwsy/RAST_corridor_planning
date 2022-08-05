/**
 * @file poly_traj_server.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief receive parametric polynomial trajectory, publish position command
 * @version 1.0
 * @date 2022-08-03
 * @copyright Copyright (c) 2022
 *
 */
#include <ros/ros.h>
#include "quadrotor_msgs/PositionCommand.h"
#include "polynomial/mini_snap_utils.hpp"
#include "traj_utils/PolyTraj.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

ros::Publisher _pos_cmd_pub, _pva_pub;

bool                  _is_traj_received = false;
int                   _traj_id;
minisnap::Trajectory _traj;
ros::Time             _t_srt;     // start time
ros::Time             _t_cur;     // current time
double                _duration;  // duration of the trajectory in seconds

double _last_yaw, _last_yaw_dot;

void polyCallback(traj_utils::PolyTrajConstPtr msg) {
  _traj_id    = msg->traj_id;
  int order   = msg->order;
  int N       = order + 1;
  int n_piece = msg->duration.size();  // number of pieces

  _duration = 0.0;
  std::vector<double> time_alloc;
  for (auto it = msg->duration.begin(); it != msg->duration.end(); ++it) {
    _duration += (*it);
    time_alloc.push_back(*it);
  }

  Eigen::VectorXd coeff;
  coeff.resize((ORDER+1)*DIM*n_piece);
  for (int i = 0; i < n_piece; i++) {
    for (int j = 0; j < ORDER + 1; j ++) {
      coeff(i*(ORDER+1)*DIM + j) = msg->coef_x[i*(ORDER+1) + j];
      coeff(i*(ORDER+1)*DIM + j + (ORDER+1)) = msg->coef_y[i*(ORDER+1) + j];
      coeff(i*(ORDER+1)*DIM + j + 2*(ORDER+1)) = msg->coef_z[i*(ORDER+1) + j];
    }
  }
  _traj.setDuration(time_alloc);
  _traj.setCoeffs(coeff);
}

// TODO: calculate yaw angle

void getYaw(const Eigen::Vector3d& p, const double &t, double &yaw, double &yaw_dot) {
  yaw = 0.0;
  yaw_dot = 0.0;
}

/**
 * @brief publish pva command for pva tracker
 * @param msg
 */
void pvaPubCallback(const ros::TimerEvent &e) {
  if (!_is_traj_received) {
    return;
  }

  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d acc;
  double yaw, yaw_dot;

  _t_cur   = ros::Time::now();
  double t = (_t_cur - _t_srt).toSec();

  if (t >= 0.0 && t < _duration) {  /* TRAJ EXEC IN PROGRESS */
    pos = _traj.getPos(t);
    vel = _traj.getVel(t);
    acc = _traj.getAcc(t);
    getYaw(pos, t, yaw, yaw_dot);
  } else if (t >= _duration) {  /* TRAJ EXEC COMPLETE */
    pos = _traj.getPos(_duration);
    vel.setZero();
    acc.setZero();
    yaw = _last_yaw;
    yaw_dot = 0;
  } else {
    ROS_ERROR("[TrajSrv]: invalid time, relative t is negative");
  }

  trajectory_msgs::JointTrajectoryPoint pva_msg;
  pva_msg.positions.push_back(pos(0));
  pva_msg.positions.push_back(pos(1));
  pva_msg.positions.push_back(pos(2));
  pva_msg.positions.push_back(yaw);
  pva_msg.velocities.push_back(vel(0));
  pva_msg.velocities.push_back(vel(1));
  pva_msg.velocities.push_back(vel(2));
  pva_msg.accelerations.push_back(acc(0));
  pva_msg.accelerations.push_back(acc(1));
  pva_msg.accelerations.push_back(acc(2));
  _pva_pub.publish(pva_msg);
}

/**
 * @brief publish quadrotor_msgs::PositionCommand for fake simulation
 *
 * @param e
 */
void cmdPubCallback(const ros::TimerEvent &e) {
  if (!_is_traj_received) {
    return;
  }
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d acc;
  double yaw, yaw_dot;

  _t_cur   = ros::Time::now();
  double t = (_t_cur - _t_srt).toSec();

  if (t >= 0.0 && t < _duration) { /* TRAJ EXEC IN PROGRESS */
    pos = _traj.getPos(t);
    vel = _traj.getVel(t);
    acc = _traj.getAcc(t);
    getYaw(pos, t, yaw, yaw_dot);
  } else if (t >= _duration) {  /* TRAJ EXEC COMPLETE */
    pos = _traj.getPos(_duration);
    vel.setZero();
    acc.setZero();
    yaw = _last_yaw;
    yaw_dot = 0;
  } else {
    ROS_ERROR("[TrajSrv]: invalid time, relative t is negative");
  }

  quadrotor_msgs::PositionCommand cmd_msg;
  cmd_msg.header.stamp    = _t_cur;
  cmd_msg.header.frame_id = "world";
  cmd_msg.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd_msg.trajectory_id   = _traj_id;

  cmd_msg.position.x     = pos(0);
  cmd_msg.position.y     = pos(1);
  cmd_msg.position.z     = pos(2);
  cmd_msg.yaw            = yaw;
  cmd_msg.velocity.x     = vel(0);
  cmd_msg.velocity.y     = vel(1);
  cmd_msg.velocity.z     = vel(2);
  cmd_msg.yaw_dot        = yaw_dot;
  cmd_msg.acceleration.x = acc(0);
  cmd_msg.acceleration.y = acc(1);
  cmd_msg.acceleration.z = acc(2);
  _pos_cmd_pub.publish(cmd_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "poly_traj_server");
  ros::NodeHandle nh("~");
  ros::Subscriber traj_sub = nh.subscribe("trajectory", 1, polyCallback);

  _pva_pub     = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);
  _pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 1);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdPubCallback);
  ros::Timer pva_timer = nh.createTimer(ros::Duration(0.01), pvaPubCallback);

  ros::Duration(2.0).sleep();
  ROS_INFO("[TrajSrv]: ready to receive trajectory");
  ros::spin();
}