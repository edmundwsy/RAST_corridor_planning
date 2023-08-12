/**
 * @file multi_eval.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2023-01-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef MULTI_EVAL_HPP
#define MULTI_EVAL_HPP

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <string>

struct Obstacle {
  /** type 0: circle, 1: cylinder, 2:box
   */
  int    type;
  double radius_x;
  double radius_y;
  double height;

  Eigen::Vector3d    pos;
  Eigen::Quaterniond q;
  Eigen::Vector2d    vel;
  Obstacle() {}
  Obstacle(int                type,
           double             width_x,
           double             width_y,
           double             height,
           Eigen::Vector3d    pos,
           Eigen::Vector2d    vel,
           Eigen::Quaterniond q)

      : type(type)
      , radius_x(width_x)
      , radius_y(width_y)
      , height(height)
      , pos(std::move(pos))
      , q(std::move(q))
      , vel(std::move(vel)) {}
};

/**
 * @brief subscribe single agents' control commands, record the desired state
 */
class SingleTrajRecorder {
 private:
  int             agent_id_;
  std::string     topic_name_;
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;

  /* data */
  double          t_;
  double          yaw_;
  double          yaw_rate_;
  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Vector3d acceleration_;

 public:
  /**
   * @brief [TODO:summary]
   *
   */
  ~SingleTrajRecorder() {}
  SingleTrajRecorder(ros::NodeHandle& nh) {
    nh_         = nh;
    agent_id_   = 0;
    topic_name_ = std::string("/uav") + std::to_string(agent_id_) +
                  std::string("/command_process/position_control");  // TODO: change to pose
  }

  SingleTrajRecorder(ros::NodeHandle& nh, int agent_id) {
    nh_         = nh;
    agent_id_   = agent_id;
    topic_name_ = std::string("/uav") + std::to_string(agent_id_) +
                  std::string("/command_process/position_control");  // TODO: change to pose
  }

  SingleTrajRecorder(ros::NodeHandle& nh, int agent_id, std::string topic_name) {
    nh_              = nh;
    agent_id_        = agent_id;
    std::string pre  = topic_name.substr(0, topic_name.find_last_of("0"));
    std::string post = topic_name.substr(topic_name.find_last_of("0") + 1);
    topic_name_      = pre + std::to_string(agent_id_) + post;
  }

  void init() {
    odom_sub_ = nh_.subscribe(topic_name_, 10, &SingleTrajRecorder::cmdCallback, this);
    ROS_INFO("subscribing to %s", topic_name_.c_str());
  }

  void cmdCallback(const nav_msgs::OdometryPtr& msg) {
    t_ = msg->header.stamp.toSec();
    position_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    velocity_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    acceleration_ << msg->twist.twist.angular.x, msg->twist.twist.angular.y,
        msg->twist.twist.angular.z;
    yaw_      = 2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    yaw_rate_ = msg->twist.twist.angular.z;
  }

  void printLog() {
    ROS_INFO(
        "[uav%i] t: %f, pos: %f, %f, %f, vel: %f, %f, %f, acc: %f, %f, %f, yaw: %f, yaw_rate: %f",
        agent_id_, t_, position_.x(), position_.y(), position_.z(), velocity_.x(), velocity_.y(),
        velocity_.z(), acceleration_.x(), acceleration_.y(), acceleration_.z(), yaw_, yaw_rate_);
  }

  Eigen::Vector3d getPosition() const { return position_; }
  Eigen::Vector3d getVelocity() const { return velocity_; }
  Eigen::Vector3d getAcceleration() const { return acceleration_; }
  double          getYaw() const { return yaw_; }
  double          getYawRate() const { return yaw_rate_; }
  double          getT() const { return t_; }

  typedef std::shared_ptr<SingleTrajRecorder> Ptr;
};

/**
 * @brief write ros log for multi agents
 */
class MultiAgentLogger {
 protected:
  int                                  num_agents_;
  std::vector<SingleTrajRecorder::Ptr> traj_recorders_;
  std::vector<Obstacle>                obstacles_;

  ros::NodeHandle nh_;
  ros::Timer      pos_query_timer_;
  ros::Subscriber obstacle_sub_;

 public:
  MultiAgentLogger() {}
  ~MultiAgentLogger() {}

  void init(ros::NodeHandle& nh) {
    nh_ = nh;
    double      pos_query_rate;
    std::string cmd_topic;
    std::string gt_topic;

    nh_.param("num_agents", num_agents_, 1);
    nh_.param("query_rate", pos_query_rate, 50.0);
    nh_.param("cmd_topic", cmd_topic, std::string("/uav0/command_process/position_control"));
    nh_.param("gt_topic", gt_topic, std::string("/gazebo/model_states"));

    ROS_INFO("num_agents: %i, query_rate: %f,  gt_topic: %s", num_agents_, pos_query_rate,
             gt_topic.c_str());

    traj_recorders_.reserve(num_agents_);
    for (int i = 0; i < num_agents_; i++) {
      traj_recorders_.push_back(std::make_shared<SingleTrajRecorder>(nh_, i, cmd_topic));
      traj_recorders_[i]->init();
    }

    obstacle_sub_    = nh.subscribe(gt_topic, 10, &MultiAgentLogger::obstacleCallback, this);
    pos_query_timer_ = nh_.createTimer(ros::Duration(1.0 / pos_query_rate),
                                       &MultiAgentLogger::queryCallback, this);
  }

  bool isActor(std::string name) {
    if (char(name[0]) == 'a' && char(name[1]) == 'c') {
      return true;
    }
    return false;
  }
  /**
   * @brief subscribe to the obstacle topic, load obstacles to a obs buffer
   *
   * @param msg msgs with obstacle ground truth states
   */
  void obstacleCallback(const gazebo_msgs::ModelStatesConstPtr& msg) {
    int n = msg->name.size();
    if ((int)obstacles_.size() < n) {
      obstacles_.clear();
      obstacles_.resize(n);
    }
    for (auto itr = msg->name.begin(); itr != msg->name.end(); itr++) {
      if (isActor(*itr)) {
        int             idx = int(itr - msg->name.begin());
        Eigen::Vector3d pos;
        Eigen::Vector2d vel2d = Eigen::Vector2d(msg->twist[idx].linear.x, msg->twist[idx].linear.y);
        Eigen::Quaterniond q(msg->pose[idx].orientation.w, msg->pose[idx].orientation.x,
                             msg->pose[idx].orientation.y, msg->pose[idx].orientation.z);
        pos << msg->pose[idx].position.x, msg->pose[idx].position.y, msg->pose[idx].position.z;
        double wx = 0.5;
        double wy = 0.5;
        double h  = 2;

        Obstacle obs(3, wx, wy, h, pos, vel2d, q);
        obstacles_[idx] = obs;
      }
    }
  }

  /**
   * @brief query the position of all agents in a fixed frequency
   *
   * @param event ros::TimerEvent
   */
  void queryCallback(const ros::TimerEvent& event) {
    for (int i = 0; i < num_agents_; i++) {
      auto            recorder  = traj_recorders_[i];
      double          t         = recorder->getT();
      double          yaw       = recorder->getYaw();
      double          yaw_rate  = recorder->getYawRate();
      Eigen::Vector3d pos       = recorder->getPosition();
      Eigen::Vector3d vel       = recorder->getVelocity();
      Eigen::Vector3d acc       = recorder->getAcceleration();
      double          min_d_obs = getMinObsDist(pos);
      double          min_d_uav = getMinUAVDist(i, pos);

      if (t > 0.0) {
        ROS_INFO(
            "[uav%i] t: %f, pos: %f, %f, %f, vel: %f, %f, %f, acc: %f, %f, %f, yaw: %f, yaw_rate: "
            "%f, "
            "min_d_obs: %f, min_d_uav: %f",
            i, t, pos.x(), pos.y(), pos.z(), vel.x(), vel.y(), vel.z(), acc.x(), acc.y(), acc.z(),
            yaw, yaw_rate, min_d_obs, min_d_uav);
      }
    }
  }

  double getMinObsDist(const Eigen::Vector3d& pos) {
    double min_d_obs = 1e3;
    for (auto obs : obstacles_) {  // cylinder
      if (obs.type == 3) {
        Eigen::Vector3d p_obs = obs.pos;
        p_obs[2]              = pos.z();

        double d = (p_obs - pos).norm() - obs.radius_x;
        if (d < min_d_obs) {
          min_d_obs = d;
        }
      } else if (obs.type == 2) {  // circle
        Eigen::Hyperplane<double, 3> plane = Eigen::Hyperplane<double, 3>::Through(
            obs.pos, obs.pos + obs.q * Eigen::Vector3d(1, 0, 0),
            obs.pos + obs.q * Eigen::Vector3d(1, 0, 0));
        Eigen::Vector3d b = plane.projection(pos);

        double vertical_dist   = plane.absDistance(pos);
        double horizontal_dist = obs.radius_x - (obs.pos - b).norm();
        double d = std::sqrt(std::pow(vertical_dist, 2) + std::pow(horizontal_dist, 2));
        if (d < min_d_obs) {
          min_d_obs = d;
        }
      }
    }
    return min_d_obs;
  }

  double getMinUAVDist(const int& id, const Eigen::Vector3d& pos) {
    double min_d_uav = 1e3;
    for (int i = 0; i < num_agents_; i++) {
      if (i == id) continue;
      double d = (traj_recorders_[i]->getPosition() - pos).norm();
      if (d < min_d_uav) {
        min_d_uav = d;
      }
    }
    return min_d_uav;
  }
};

#endif  // MULTI_EVAL_HPP
