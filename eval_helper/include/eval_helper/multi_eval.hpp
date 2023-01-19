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

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <string>
#include "quadrotor_msgs/PositionCommand.h"

class SingleTrajRecorder {
 private:
  int             agent_id_;
  std::string     topic_name_;
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;

  /* data */
  double          t_;
  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Vector3d acceleration_;
  double          yaw_;
  double          yaw_rate_;

 public:
  ~SingleTrajRecorder() {}
  SingleTrajRecorder(ros::NodeHandle& nh) {
    nh_         = nh;
    agent_id_   = 0;
    topic_name_ = std::string("/uav") + std::to_string(agent_id_) +
                  std::string("/controller/pos_cmd");  // TODO: change to pose
  }

  SingleTrajRecorder(ros::NodeHandle& nh, int agent_id) {
    nh_         = nh;
    agent_id_   = agent_id;
    topic_name_ = std::string("/uav") + std::to_string(agent_id_) +
                  std::string("/controller/pos_cmd");  // TODO: change to pose
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

  void cmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    t_                = msg->header.stamp.toSec();
    position_.x()     = msg->position.x;
    position_.y()     = msg->position.y;
    position_.z()     = msg->position.z;
    velocity_.x()     = msg->velocity.x;
    velocity_.y()     = msg->velocity.y;
    velocity_.z()     = msg->velocity.z;
    acceleration_.x() = msg->acceleration.x;
    acceleration_.y() = msg->acceleration.y;
    acceleration_.z() = msg->acceleration.z;
    yaw_              = msg->yaw;
    yaw_rate_         = msg->yaw_dot;
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

class MultiAgentLogger {
 protected:
  int                                  num_agents_;
  std::vector<SingleTrajRecorder::Ptr> traj_recorders_;

  ros::NodeHandle nh_;
  ros::Timer      pos_query_timer_;

 public:
  MultiAgentLogger() {}
  ~MultiAgentLogger() {}

  void init(ros::NodeHandle& nh) {
    nh_ = nh;
    nh_.param("num_agents", num_agents_, 1);
    double pos_query_rate;
    nh_.param("query_rate", pos_query_rate, 50.0);
    std::string topic_name;
    nh_.param("topic_name", topic_name, std::string("/uav0/controller/pos_cmd"));

    traj_recorders_.reserve(num_agents_);
    for (int i = 0; i < num_agents_; i++) {
      traj_recorders_.push_back(std::make_shared<SingleTrajRecorder>(nh_, i, topic_name));
      traj_recorders_[i]->init();
    }

    pos_query_timer_ = nh_.createTimer(ros::Duration(1.0 / pos_query_rate),
                                       &MultiAgentLogger::queryCallback, this);
  }

  /**
   * @brief query the position of all agents in a fixed frequency
   *
   * @param event ros::TimerEvent
   */
  void queryCallback(const ros::TimerEvent& event) {
    for (auto& recorder : traj_recorders_) {
      recorder->printLog();
    }
    printRelativeDistance();
  }

  void printRelativeDistance() {
    std::vector<Eigen::Vector3d> positions;
    for (auto& recorder : traj_recorders_) {
      positions.push_back(recorder->getPosition());
    }
    double min_dist = 1000.0;
    for (int i = 0; i < num_agents_; i++) {
      for (int j = i + 1; j < num_agents_; j++) {
        double distance = (positions[i] - positions[j]).norm();
        if (distance < min_dist) {
          min_dist = distance;
        }
      }
    }
    ROS_INFO("Minimum distance: %f", min_dist);
  }
};

#endif  // MULTI_EVAL_HPP
