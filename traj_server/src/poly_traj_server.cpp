/**
 * @file poly_traj_server.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief receive parametric polynomial trajectory, publish position command
 * @version 1.0
 * @date 2022-08-03
 * @copyright Copyright (c) 2022
 *
 */
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <Eigen/Eigen>

#include "quadrotor_msgs/PositionCommand.h"
#include "traj_utils/PolyTraj.h"
#include "traj_utils/poly_traj.hpp"
#include "trajectory_msgs/JointTrajectoryPoint.h"

ros::Publisher _pos_cmd_pub, _pva_pub, _vis_pub;

bool _is_traj_received = false;
bool _is_triggered     = false;

int                    _traj_id;
polynomial::Trajectory _traj;
ros::Time              _t_str;     // start time
ros::Time              _t_cur;     // current time
double                 _duration;  // duration of the trajectory in seconds

double _last_yaw, _last_yaw_dot;

/**
 * @brief receive trigger message, start the trajectory
 * @param msg
 */
void triggerCallback(const geometry_msgs::PoseStampedPtr &msg) {
  ROS_WARN("[TrajSrv] trigger received");
  _is_triggered = true;
}

/**
 * @brief recieve parametric polynomial trajectory
 *
 * @param msg
 */
void polyCallback(traj_utils::PolyTrajConstPtr msg) {
  _traj_id    = msg->traj_id;
  int order   = msg->order;
  int N       = order + 1;
  int n_piece = msg->duration.size();  // number of pieces

  _t_str = msg->start_time;

  _duration = 0.0;
  std::vector<double> time_alloc;
  for (auto it = msg->duration.begin(); it != msg->duration.end(); ++it) {
    _duration += (*it);
    time_alloc.push_back(*it);
  }

  Eigen::VectorXd coeff;
  coeff.resize(N * DIM * n_piece);
  for (int i = 0; i < n_piece; i++) {
    for (int j = 0; j < ORDER + 1; j++) {
      coeff[i * N * DIM + j]         = msg->coef_x[i * N + j];
      coeff[i * N * DIM + j + N]     = msg->coef_y[i * N + j];
      coeff[i * N * DIM + j + 2 * N] = msg->coef_z[i * N + j];
    }
  }

  if (time_alloc.size() > 0) {
    _traj.setDuration(time_alloc);
    _traj.setCoeffs(coeff);
    _is_traj_received = true; /** only when traj is valid */
  } else {
    _is_traj_received = false;
  }
}

// TODO: calculate yaw angle

void getYaw(const Eigen::Vector3d &p, const double &t, double &yaw, double &yaw_dot) {
  yaw     = 0.0;
  yaw_dot = 0.0;
}

void publishPVA(const Eigen::Vector3d &pos,
                const Eigen::Vector3d &vel,
                const Eigen::Vector3d &acc,
                const double &         yaw,
                const double &         yaw_dot) {
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

void publishCmd(const Eigen::Vector3d &pos,
                const Eigen::Vector3d &vel,
                const Eigen::Vector3d &acc,
                const double &         yaw,
                const double &         yaw_dot) {
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

/**
 * @brief publish quadrotor_msgs::PositionCommand for fake simulation
 *
 * @param e
 */
void PubCallback(const ros::TimerEvent &e) {
  if (!_is_traj_received) {
    ROS_INFO_ONCE("[TrajSrv] waiting for trajectory");
    return;
  } else if (!_is_triggered) {
    ROS_INFO_ONCE("[TrajSrv] waiting for trigger");
    return;
  }

  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d acc;
  double          yaw, yaw_dot;

  _t_cur   = ros::Time::now();
  double t = (_t_cur - _t_str).toSec();

  if (t >= 0.0 && t < _duration) { /* TRAJ EXEC IN PROGRESS */
    pos = _traj.getPos(t);
    vel = _traj.getVel(t);
    acc = _traj.getAcc(t);
    getYaw(pos, t, yaw, yaw_dot);
  } else if (t >= _duration) { /* TRAJ EXEC COMPLETE */
    ROS_WARN_ONCE("[TrajSrv] trajectory execution complete");
    pos = _traj.getPos(_duration);
    vel.setZero();
    acc.setZero();
    yaw     = _last_yaw;
    yaw_dot = 0;
  } else {
    ROS_ERROR_ONCE("[TrajSrv]: invalid time, relative t is negative");
  }
  publishPVA(pos, vel, acc, yaw, yaw_dot);
  publishCmd(pos, vel, acc, yaw, yaw_dot);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "poly_traj_server");
  ros::NodeHandle nh("~");
  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), PubCallback);
  ros::Subscriber traj_sub    = nh.subscribe("trajectory", 1, polyCallback);
  ros::Subscriber trigger_sub = nh.subscribe("/traj_start_trigger", 10, triggerCallback);

  _pva_pub     = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);
  _pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 1);

  ros::Duration(3.0).sleep();
  ROS_INFO("[TrajSrv]: ready to receive trajectory");
  ros::spin();
  return 0;
}