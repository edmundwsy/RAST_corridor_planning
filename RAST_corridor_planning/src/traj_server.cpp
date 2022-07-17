/**
 * @file traj_server.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief subscribe parametric polynomial trajectories, publish as position commands
 * @version 1.0
 * @date 2022-07-17
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <CorridorMiniSnap/corridor_minisnap.h>
#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
// #include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

using namespace Eigen;

// TODO(siyuanwu): use our own traj_opt namespace
// TOOD(siyuanwu): use corridor minisnap first, use MINCO and other methods later

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {0, 0, 0};
// double vel_gain[3] = {0, 0, 0};

bool receive_traj_ = false;
// boost::shared_ptr<poly_traj::Trajectory> traj_;
std::shared_ptr<traj_opt::Trajectory>    traj_;
double                                   traj_duration_;
// ros::Time start_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;

// yaw control
double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double time_forward_;

traj_utils::PolyTraj trajMsg_, trajMsg_last_;

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1
                        ? atan2(dir(1), dir(0))
                        : last_yaw_;

  double yawdot = 0;
  double d_yaw = yaw_temp - last_yaw_;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2 * M_PI;
  }

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    // yawdot = last_yawdot_ + dt * YDDM;
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    // yawdot = YDM;
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  yawdot = d_yaw / dt;

  double yaw = last_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;

  yaw_yawdot.second = yaw_temp;

  return yaw_yawdot;
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

  last_pos_ = p;
}

bool execTraj(traj_utils::PolyTraj &trajMsg) {
  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - trajMsg.start_time).toSec();
  if (t_cur > 0) {
    // trajMsg
    if (trajMsg.order != 5)
    {
      ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
      return false;
    }
    if (trajMsg.duration.size() * (trajMsg.order + 1) != trajMsg.coef_x.size())
    {
      ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
      return false;
    }

    int piece_nums = trajMsg.duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i)
    {
      int i6 = i * 6;
      cMats[i].row(0) << trajMsg.coef_x[i6 + 0], trajMsg.coef_x[i6 + 1], trajMsg.coef_x[i6 + 2],
          trajMsg.coef_x[i6 + 3], trajMsg.coef_x[i6 + 4], trajMsg.coef_x[i6 + 5];
      cMats[i].row(1) << trajMsg.coef_y[i6 + 0], trajMsg.coef_y[i6 + 1], trajMsg.coef_y[i6 + 2],
          trajMsg.coef_y[i6 + 3], trajMsg.coef_y[i6 + 4], trajMsg.coef_y[i6 + 5];
      cMats[i].row(2) << trajMsg.coef_z[i6 + 0], trajMsg.coef_z[i6 + 1], trajMsg.coef_z[i6 + 2],
          trajMsg.coef_z[i6 + 3], trajMsg.coef_z[i6 + 4], trajMsg.coef_z[i6 + 5];

      dura[i] = trajMsg.duration[i];
    }

    traj_.reset(new traj_opt::Trajectory(dura, cMats));  // TODO: check if it works
    traj_duration_ = traj_->getDuration();

    // publish_cmd
    if (t_cur > traj_duration_) {
      ROS_ERROR_ONCE("[traj_server] trajectory too short left!");
      return false;
    }
    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
    std::pair<double, double> yaw_yawdot(0, 0);

    static ros::Time time_last = ros::Time::now();
    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
      pos = traj_->getPos(t_cur);
      vel = traj_->getVel(t_cur);
      acc = traj_->getAcc(t_cur);
      jer = traj_->getJrk(t_cur);

      /*** calculate yaw ***/
      yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());
      /*** calculate yaw ***/

      time_last = time_now;
      last_yaw_ = yaw_yawdot.first;
      last_pos_ = pos;

      // publish
      publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);

      return true;
    }

  }

  return false;

}

void polyTrajCallback(traj_utils::PolyTrajPtr msg) {
  trajMsg_ = *msg;
  // ROS_WARN("[traj server]Receive trajectory and start time is %d.", msg->start_time);
  if (!receive_traj_) {
    trajMsg_last_ = trajMsg_;
    receive_traj_ = true;
  }
}

void cmdCallback(const ros::TimerEvent &e) {
  if (!receive_traj_) {
    return;
  }

  // if ((ros::Time::now() - heartbeat_time_).toSec() > 0.5)
  // {
  //   ROS_ERROR("[traj_server] Lost heartbeat from the planner, is he dead?");

  //   receive_traj_ = false;
  //   publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
  // }

  if (execTraj(trajMsg_)) {
    trajMsg_last_ = trajMsg_;
    return;
  } 
  else if (execTraj(trajMsg_last_)) {
    return;
  }
  ROS_ERROR_ONCE("[traj server] traj received invalid!");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);
  // TODO(siyuan): figure out what is heartbeat
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}