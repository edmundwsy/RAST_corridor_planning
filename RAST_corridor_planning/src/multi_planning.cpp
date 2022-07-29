/**
 * @file multi_planning.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-07-29
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <multi_planning.h>

namespace planner {
Planner::Planner(const PlannerConfig conf, ros::NodeHandle &nh) : _config(conf), _nh(nh) {
  _astar_planner.setTimeParameters(_config.a_star_search_time_step, _config.planning_time_step);
  _astar_planner.setHeightLimit(_config.use_height_limit, _config.height_limit_max,
                                _config.height_limit_min);
  _astar_planner.setIfSampleZDirection(_config.sample_z_acc);
  _astar_planner.setMaximumVelAccAndStep(
      static_cast<float>(_config.max_vel), static_cast<float>(_config.max_vel),
      static_cast<float>(_config.max_acc), static_cast<float>(_config.max_acc / 2.0),
      _config.a_star_acc_sample_step);
  _astar_planner.setRiskThreshold(_config.risk_threshold_motion_primitive,
                                  _config.risk_threshold_single_voxel,
                                  _config.risk_threshold_corridor);

  /*** INITIALIZE VISUALIZATION ***/
  std::string frame_id = "world";
  vis_.reset(new Visualizer(_nh, frame_id));

  /*** SUBSCRIBERS ***/
  _future_risk_sub =
      _nh.subscribe("/my_map/future_risk_full_array", 1, &Planner::FutureRiskCallback, this);
  _pose_sub = _nh.subscribe("/mavros/local_position/pose", 1, &Planner::PoseCallback, this);
  _vel_sub = _nh.subscribe("/mavros/local_position/velocity_local", 1, &Planner::VelCallback, this);

  /*** PUBLISHERS ***/
  _traj_pub = _nh.advertise<traj_utils::PolyTraj>("~trajectory", 1);

  ROS_INFO("Wait for 2 seconds");
  ros::Duration(2.0).sleep();

  _traj_timer =
      _nh.createTimer(ros::Duration(_config.planning_time_step), &Planner::TrajTimerCallback, this);

  // ros::AsyncSpinner spinner(3);
  // spinner.start();
  // ros::waitForShutdown();
}

Planner::FutureRiskCallback(const decomp_ros_msgs::DynPolyhedronArrayConstPtr &risk_msg) {
  _is_future_risk_locked = true;
  for (int i = 0; i < VOXEL_NUM; ++i) {
    for (int j = 0; j < RISK_MAP_NUMBER; ++j) {
      _future_risk[i][j] = risk_msg->data[i * risk_msg->layout.dim[0].stride + j];
    }
  }
  _is_future_risk_locked = false;

  _pose.pose.position.x = risk_msg->data[VOXEL_NUM * RISK_MAP_NUMBER];
  _pose.pose.position.y = risk_msg->data[VOXEL_NUM * RISK_MAP_NUMBER + 1];
  _pose.pose.position.z = risk_msg->data[VOXEL_NUM * RISK_MAP_NUMBER + 2];

  _is_future_risk_locked = true;
}

Planner::PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
  if (!_is_state_locked) {
    _is_state_locked     = true;
    _pos.x() = msg.pose.position.x;
    _pos.y() = msg.pose.position.y;
    _pos.z() = msg.pose.position.z;

    _att.x() = msg.pose.orientation.x;
    _att.y() = msg.pose.orientation.y;
    _att.z() = msg.pose.orientation.z;
    _att.w() = msg.pose.orientation.w;

    position_queue_.push(_pos);
    quaternion_queue_.push(_att);
    pose_att_time_queue.push(msg.header.stamp.toSec());

    _is_position_received = true;
  }

  _is_state_locked = false;

  Eigen::Quaternionf axis;  //= quad * q1 * quad.inverse();
  axis.w()                       = cos(-M_PI / 4.0);
  axis.x()                       = 0.0;
  axis.y()                       = 0.0;
  axis.z()                       = sin(-M_PI / 4.0);
  Eigen::Quaternionf rotated_att = _att * axis;
}

void Planner::VelCallback(const geometry_msgs::TwistStamped& msg) {
  _vel.x() = msg.twist.linear.x;
  _vel.y() = msg.twist.linear.y;
  _vel.z() = msg.twist.linear.z;

  /** Calculate virtual accelerates from velocity. Original accelerates given by px4 is too noisy
   * **/
  static bool   init_v_flag = true;
  static double last_time, last_vx, last_vy, last_vz;

  if (init_v_flag) {
    init_v_flag = false;
  } else {
    double delt_t   = ros::Time::now().toSec() - last_time;
    current_acc_(0) = (_vel(0) - last_vx) / delt_t;
    current_acc_(1) = (_vel(1) - last_vy) / delt_t;
    current_acc_(2) = (_vel(2) - last_vz) / delt_t;

    if (fabs(current_acc_(0)) < 0.2) current_acc_(0) = 0.0;  // dead zone for acc x
    if (fabs(current_acc_(1)) < 0.2) current_acc_(1) = 0.0;  // dead zone for acc y
    if (fabs(current_acc_(2)) < 0.2) current_acc_(2) = 0.0;  // dead zone for acc z

    for (int i = 0; i < 3; i++) {
      if (current_acc_(i) < -max_differentiated_current_a) {
        current_acc_(i) = -max_differentiated_current_a;
      } else if (current_acc_(i) > max_differentiated_current_a) {
        current_acc_(i) = max_differentiated_current_a;
      }
    }
    // ROS_INFO("acc=(%f, %f, %f)", current_acc_(0), current_acc_(1),
    // current_acc_(2));
  }
  last_time = ros::Time::now().toSec();
  last_vx   = _vel(0);
  last_vy   = _vel(1);
  last_vz   = _vel(2);
}

}  // namespace planner
