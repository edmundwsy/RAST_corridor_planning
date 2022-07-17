#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <traj_utils/MINCOTraj.h>
#include <traj_utils/GoalSet.h>
#include <sensor_msgs/PointCloud2.h>
#include <unistd.h>
#include "reliable_bridge.hpp"

using namespace std;

std::vector<int> id_list_;
std::vector<string> ip_list_;
ros::Subscriber other_odoms_sub_, one_traj_sub_, mandatory_stop_sub_, goal_sub_;
ros::Publisher other_odoms_pub_, one_traj_pub_, mandatory_stop_pub_, goal_pub_;
ros::Subscriber goal_exploration_sub_,star_cvx_sub_,frontier_sub_;
ros::Publisher goal_exploration_pub_,star_cvx_pub_,frontier_pub_;
int self_id_;
int self_id_in_bridge_;
int drone_num_;
int ground_station_num_;
double odom_broadcast_freq_;
bool is_groundstation_;

unique_ptr<ReliableBridge> bridge;

inline int remap_ground_station_id(int id)
{
  return id+drone_num_;
}

template <typename T>
int send_to_all_drone(string topic, T &msg)
{
  int err_code = 0; 
  for (int i = 0; i < drone_num_; ++i)// Only send to all drones.
  {
    if (i == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    err_code += bridge->send_msg_to_one(i,topic,msg);
    if(err_code< 0)
    {
      ROS_ERROR("[Bridge] SEND ERROR %s !!",typeid(T).name());
    }
  }
  return err_code;
}

template <typename T>
int send_to_all_groundstation(string topic, T &msg)
{
  int err_code = 0;
  for (int i = 0; i < ground_station_num_; ++i)// Only send to all groundstations.
  {
    int ind = remap_ground_station_id(i);
    if (ind == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    err_code += bridge->send_msg_to_one(ind,topic,msg);
    if(err_code < 0)
    {
      ROS_ERROR("[Bridge] SEND ERROR %s !!",typeid(T).name());
    }
  }
  return err_code;
}

void register_callbak_to_all_groundstation(string topic_name, function<void(int, ros::SerializedMessage &)> callback)
{
  for (int i = 0; i < ground_station_num_; ++i)
  {
    int ind = remap_ground_station_id(i);
    if (ind == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    bridge->register_callback(ind,topic_name,callback);
  }
}

void register_callbak_to_all_drones(string topic_name, function<void(int, ros::SerializedMessage &)> callback)
{
  for (int i = 0; i < drone_num_; ++i)
  {
    if (i == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    bridge->register_callback(i,topic_name,callback);
  }
}
 

// Here is callback from local topic.
void odom_sub_udp_cb(const nav_msgs::OdometryPtr &msg)
{

  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() * odom_broadcast_freq_ < 1.0)
  {
    return;
  }
  t_last = t_now;

  msg->child_frame_id = string("drone_") + std::to_string(self_id_);

  send_to_all_drone("/odom",*msg);// Only send to drones.
  send_to_all_groundstation("/odom",*msg);// Only send to ground stations.
}

void one_traj_sub_udp_cb(const traj_utils::MINCOTrajPtr &msg)
{
  one_traj_pub_.publish(msg);  // Send to myself.
  if (bridge->send_msg_to_all("/traj_from_planner",*msg))
  {
    ROS_ERROR("[Bridge] SEND ERROR (ONE_TRAJ)!!!");
  }
}

void mandatory_stop_sub_udp_cb(const std_msgs::EmptyPtr &msg)
{
  send_to_all_drone("/stop",*msg);
}

void goal_sub_udp_cb(const traj_utils::GoalSetPtr &msg)
{
  if (msg->drone_id==self_id_in_bridge_)
  {
    goal_pub_.publish(msg);  // Send to myself.
    return;
  }
  
  if(bridge->send_msg_to_one(msg->drone_id,"/goal",*msg)< 0)
  {
    ROS_ERROR("[Bridge] SEND ERROR (GOAL)!!!");
  }
}

void goal_exploration_sub_udp_cb(const traj_utils::GoalSetPtr &msg)
{
  if (bridge->send_msg_to_all("/goal_exploration",*msg))
  {
    ROS_ERROR("[Bridge] SEND ERROR (goal_exploration)!!!");
  }

}

void star_cvx_sub_udp_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
  if (bridge->send_msg_to_all("/star_cvx",*msg))
  {
    ROS_ERROR("[Bridge] SEND ERROR (star_cvx)!!!");
  }

}

void frontier_sub_udp_cb(const sensor_msgs::PointCloud2Ptr &msg)
{

  if (bridge->send_msg_to_all("/frontier",*msg))
  {
    ROS_ERROR("[Bridge] SEND ERROR (frontier)!!!");
  }

}


// Here is callback when the brodge received the data from others.
void odom_bridge_cb(int ID, ros::SerializedMessage& m)
{
  nav_msgs::Odometry odom_msg_;
  ros::serialization::deserializeMessage(m,odom_msg_);
  other_odoms_pub_.publish(odom_msg_);
}

void goal_bridge_cb(int ID, ros::SerializedMessage& m)
{
  traj_utils::GoalSet goal_msg_;
  ros::serialization::deserializeMessage(m,goal_msg_);
  goal_pub_.publish(goal_msg_);
}

void traj_bridge_cb(int ID, ros::SerializedMessage& m)
{
  traj_utils::MINCOTraj MINCOTraj_msg_;
  ros::serialization::deserializeMessage(m,MINCOTraj_msg_);
  one_traj_pub_.publish(MINCOTraj_msg_);
}

void stop_bridge_cb(int ID, ros::SerializedMessage& m)
{
  std_msgs::Empty stop_msg_;
  ros::serialization::deserializeMessage(m,stop_msg_);
  mandatory_stop_pub_.publish(stop_msg_);
}


void goal_exploration_bridge_cb(int ID, ros::SerializedMessage& m)
{
  traj_utils::GoalSet goal_msg_;
  ros::serialization::deserializeMessage(m,goal_msg_);
  goal_exploration_pub_.publish(goal_msg_);
}

void star_cvx_bridge_cb(int ID, ros::SerializedMessage& m)
{
  sensor_msgs::PointCloud2 point_msg_;
  ros::serialization::deserializeMessage(m,point_msg_);
  star_cvx_pub_.publish(point_msg_);
}

void frontier_bridge_cb(int ID, ros::SerializedMessage& m)
{
  sensor_msgs::PointCloud2 point_msg_;
  ros::serialization::deserializeMessage(m,point_msg_);
  frontier_pub_.publish(point_msg_);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");

  // nh.param("broadcast_ip", udp_ip_, string("127.0.0.255"));
  nh.param("self_id", self_id_, -1);
  nh.param("is_ground_station", is_groundstation_, false);
  nh.param("drone_num", drone_num_, 0);
  nh.param("ground_station_num", ground_station_num_, 0);
  nh.param("odom_max_freq", odom_broadcast_freq_, 1000.0);

  id_list_.resize(drone_num_ + ground_station_num_);
  ip_list_.resize(drone_num_ + ground_station_num_);
  for (int i = 0; i < drone_num_ + ground_station_num_; ++i)
  {
    nh.param((i < drone_num_ ? "drone_ip_" + to_string(i) : "ground_station_ip_" + to_string(i-drone_num_)), ip_list_[i], string("127.0.0.1"));
    id_list_[i]=i;
  }  
  self_id_in_bridge_ = self_id_;
  if (is_groundstation_)
  {
    self_id_in_bridge_ = remap_ground_station_id(self_id_);
  }
  //the ground statation ID = self ID + drone_num_

  if (self_id_in_bridge_ < 0 || self_id_in_bridge_ > 99)
  {
    ROS_WARN("[swarm bridge] Wrong self_id!");
    exit(EXIT_FAILURE);
  }

  //initalize the bridge  
  bridge.reset(new ReliableBridge(self_id_in_bridge_,ip_list_,id_list_,100000));    
  
  other_odoms_sub_ = nh.subscribe("my_odom", 10, odom_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  other_odoms_pub_ = nh.advertise<nav_msgs::Odometry>("/others_odom", 10);
  //register callback
  register_callbak_to_all_drones("/odom",odom_bridge_cb);

  one_traj_sub_ = nh.subscribe("/broadcast_traj_from_planner", 100, one_traj_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  one_traj_pub_ = nh.advertise<traj_utils::MINCOTraj>("/broadcast_traj_to_planner", 100);
  bridge->register_callback_for_all("/traj_from_planner",traj_bridge_cb);

  mandatory_stop_sub_ = nh.subscribe("/mandatory_stop_from_users", 100, mandatory_stop_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  mandatory_stop_pub_ = nh.advertise<std_msgs::Empty>("/mandatory_stop_to_planner", 100);
  register_callbak_to_all_groundstation("/stop",stop_bridge_cb);
  

  goal_sub_ = nh.subscribe("/goal_user2brig", 100, goal_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  goal_pub_ = nh.advertise<traj_utils::GoalSet>("/goal_brig2plner", 100);
  register_callbak_to_all_groundstation("/goal",goal_bridge_cb);

  goal_exploration_sub_ = nh.subscribe("/goal_with_id", 100, goal_exploration_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  goal_exploration_pub_ = nh.advertise<traj_utils::GoalSet>("/goal_with_id_to_planner", 100);
  bridge->register_callback_for_all("/goal_exploration",goal_exploration_bridge_cb);

  star_cvx_sub_ = nh.subscribe("/free_map/star_cvx", 100, star_cvx_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  star_cvx_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/free_map/star_cvx_to_planner", 100);
  bridge->register_callback_for_all("/star_cvx",star_cvx_bridge_cb);

  frontier_sub_ = nh.subscribe("/frontier_pc", 100, frontier_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  frontier_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/frontier_pc_to_planner", 100);
  bridge->register_callback_for_all("/frontier",frontier_bridge_cb);

  ros::spin();

  return 0;
}
