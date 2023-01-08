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

class SingleTrajRecorder {
 private:
  int                 agent_id_;
  std::string         topic_name_;
  geometry_msgs::Pose pose_;
  ros::Subscriber     odom_sub_;

 public:
  SingleTrajRecorder(ros::NodeHandle& nh) {
    agent_id_   = 0;
    topic_name_ = std::string("/uav") + std::to_string(agent_id_) +
                  std::string("/mavros/local_position/odom");  // TODO: change to pose
    odom_sub_ = nh.subscribe(topic_name_, 10, &SingleTrajRecorder::poseCallback, this);
    ROS_INFO("subscribing to %s", topic_name_.c_str());
  }

  SingleTrajRecorder(ros::NodeHandle& nh, int agent_id) {
    agent_id_   = agent_id;
    topic_name_ = std::string("/uav") + std::to_string(agent_id_) +
                  std::string("/mavros/local_position/odom");  // TODO: change to pose
    odom_sub_ = nh.subscribe(topic_name_, 10, &SingleTrajRecorder::poseCallback, this);
    ROS_INFO("subscribing to %s", topic_name_.c_str());
  }

  ~SingleTrajRecorder() {}

  void poseCallback(const geometry_msgs::PoseStamped& msg) {
    // pose_ = msg.pose;
    pose_.position.x    = msg.pose.position.x;
    pose_.position.y    = msg.pose.position.y;
    pose_.position.z    = msg.pose.position.z;
    pose_.orientation.x = msg.pose.orientation.x;
    pose_.orientation.y = msg.pose.orientation.y;
    pose_.orientation.z = msg.pose.orientation.z;
  }

  void getPosition(geometry_msgs::Point& position) {
    position.x = pose_.position.x;
    position.y = pose_.position.y;
    position.z = pose_.position.z;
  }

  typedef std::shared_ptr<SingleTrajRecorder> Ptr;
};

class MultiAgentEvaluation {
 private:
  int                                  num_agents_;
  std::vector<SingleTrajRecorder::Ptr> traj_recorders_;

  ros::NodeHandle nh_;
  ros::Timer      pos_query_timer_;

 public:
  MultiAgentEvaluation() {}
  ~MultiAgentEvaluation() {}

  void init(ros::NodeHandle& nh) {
    nh_ = nh;
    nh_.param("num_agents", num_agents_, 1);
    double pos_query_rate;
    nh_.param("pos_query_rate", pos_query_rate, 50.0);

    traj_recorders_.reserve(num_agents_);
    for (int i = 0; i < num_agents_; i++) {
      traj_recorders_.push_back(std::make_shared<SingleTrajRecorder>(nh_, i));
    }

    pos_query_timer_ = nh_.createTimer(ros::Duration(1.0 / pos_query_rate),
                                       &MultiAgentEvaluation::posQueryCallback, this);
  }

  void posQueryCallback(const ros::TimerEvent& event) {
    std::vector<Eigen::Vector3d> positions;
    positions.resize(num_agents_);
    for (int i = 0; i < num_agents_; i++) {
      geometry_msgs::Point position;
      traj_recorders_[i]->getPosition(position);
      positions[i] << position.x, position.y, position.z;
    }

    double min_dist = 1000.0;
    for (int i = 0; i < num_agents_; i++) {
      for (int j = i + 1; j < num_agents_; j++) {
        double dist = (positions[i] - positions[j]).norm();
        ROS_INFO("distance between agent %d and agent %d is %f", i, j, dist);
        if (dist < min_dist) {
          min_dist = dist;
        }
      }
    }

    ROS_INFO("min distance is %f", min_dist);
  }
};

#endif  // MULTI_EVAL_HPP
