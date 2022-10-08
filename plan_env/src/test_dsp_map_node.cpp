/**
 * @file test_dsp_map_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <plan_env/dsp_map_new.h>
// #include <plan_env/dsp_map.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include "quadrotor_msgs/PositionCommand.h"

#include <Eigen/Eigen>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
/** include from grid_map */
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>

/* ROS Utilities */
ros::Subscriber click_sub_;
ros::Publisher  cmd_pub_;
ros::Publisher  cloud_pub_;

DSPMapStaticV2::Ptr dsp_map_;
float               risk_maps_[VOXEL_NUM][3];
float               valid_clouds_[5000 * 3];

Eigen::Vector3d end_pos_, start_pos_, cur_pos_, dir_;
double          dist_;
double          vel_ = 1.0;  // m/s
double          dt_  = 0.1;  // s

/** sync topics */
bool is_pose_sub_ = false;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                        nav_msgs::Odometry>
    SyncPolicyCloudOdom;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                        geometry_msgs::PoseStamped>
                                                                            SyncPolicyCloudPose;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>>         odom_sub_;
std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>   cloud_sub_;

SynchronizerCloudOdom sync_cloud_odom_;
SynchronizerCloudPose sync_cloud_pose_;

/* Functions */
inline bool inRange(const Eigen::Vector3f &p) {
  if (p.x() > -6.5F && p.x() < 6.5F && p.y() > -6.5F && p.y() < 6.5F && p.z() > -6.5F &&
      p.z() < 6.5F) {
    return true;
  }
  return false;
}

/**
 * @brief set goal position
 *
 * @param msg
 */
void clickCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  end_pos_(0) = msg->pose.position.x;
  end_pos_(1) = msg->pose.position.y;
  end_pos_(2) = 1;

  dir_  = end_pos_ - cur_pos_;
  dir_  = dir_ / dir_.norm();
  dist_ = (end_pos_ - start_pos_).norm();
}

/**
 * @brief update current position
 *
 * @param event
 */
void moveCallback(const ros::TimerEvent &event) {
  if ((end_pos_ - cur_pos_).norm() < 0.1) {
    return;
  }
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp    = ros::Time::now();
  cmd.header.frame_id = "world";

  cur_pos_ = cur_pos_ + dir_ * vel_ * dt_;

  cmd.position.x = cur_pos_.x();
  cmd.position.y = cur_pos_.y();
  cmd.position.z = cur_pos_.z();

  cmd_pub_.publish(cmd);
}

/**
 * @brief publish point cloud
 *
 */
void publishMap() {
  int                            num_occupied = 0;
  clock_t                        t1           = clock();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  dsp_map_->getOccupancyMapWithRiskMaps(num_occupied, cloud, &risk_maps_[0][0], 0.2);
  for (auto &p : cloud.points) {
    p.x += cur_pos_.x();
    p.y += cur_pos_.y();
    p.z += cur_pos_.z();
  }
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp    = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  cloud_pub_.publish(cloud_msg);
  clock_t t2 = clock();
  std::cout << "num_occupied: " << num_occupied << std::endl;
  std::cout << "time: " << (t2 - t1) * 1000 / (double)CLOCKS_PER_SEC << "ms" << std::endl;
}

/**
 * @brief publish point clouds in a fixed frequency 
 *
 * @param event
 */
void pubCallback(const ros::TimerEvent &event) { publishMap(); }

/**
 * @brief subscribe point cloud and odometry
 *
 * @param cloud_msg
 * @param odom_msg
 */
void cloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                       const nav_msgs::Odometry::ConstPtr &      odom_msg) {
  Eigen::Vector3f    pos(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                      odom_msg->pose.pose.position.z);
  Eigen::Quaternionf q(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                       odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
  Eigen::Matrix3f    R = q.toRotationMatrix();
  double             t = cloud_msg->header.stamp.toSec();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *cloud_in);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in);
  sor.setLeafSize(0.2, 0.2, 0.2);
  sor.filter(*cloud_filtered);

  /* filter points which are too far from the drone */
  int n_valid = 0;  // number of valid points
  for (int i = 0; i < cloud_filtered->points.size(); i++) {
    float           x = cloud_filtered->points[i].z;
    float           y = -cloud_filtered->points[i].x;
    float           z = -cloud_filtered->points[i].y;
    Eigen::Vector3f p(x, y, z);
    Eigen::Vector3f p_w = R * p;
    if (inRange(p)) {
      valid_clouds_[n_valid * 3 + 0] = x;
      valid_clouds_[n_valid * 3 + 1] = y;
      valid_clouds_[n_valid * 3 + 2] = z;
      n_valid++;
      if (n_valid >= 5000) {
        break;
      }
    }
  }
  clock_t t_update_0 = clock();
  std::cout << "number of valid points: " << n_valid << std::endl;
  if (!dsp_map_->update(n_valid, 3, valid_clouds_, pos.x(), pos.y(), pos.z(), t, q.w(), q.x(),
                        q.y(), q.z())) {
    return;
  }
  clock_t t_update_1 = clock();
  double  duration   = static_cast<double>((t_update_1 - t_update_0) * 1000 / CLOCKS_PER_SEC);
  std::cout << "update time: " << duration << "ms" << std::endl;
}

/**
 * @brief subscribe point cloud and odometry
 *
 * @param cloud_msg
 * @param pose_msg
 */
void cloudPoseCallback(const sensor_msgs::PointCloud2::ConstPtr &  cloud_msg,
                       const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
  Eigen::Vector3f    pos(pose_msg->pose.position.x, pose_msg->pose.position.y,
                      pose_msg->pose.position.z);
  Eigen::Quaternionf q(pose_msg->pose.orientation.w, pose_msg->pose.orientation.x,
                       pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);
  Eigen::Matrix3f    R = q.toRotationMatrix();
  double             t = cloud_msg->header.stamp.toSec();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *cloud_in);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in);
  sor.setLeafSize(0.2, 0.2, 0.2);
  sor.filter(*cloud_filtered);

  /* filter points which are too far from the drone */
  int n_valid = 0;  // number of valid points
  for (int i = 0; i < cloud_filtered->points.size(); i++) {
    float           x = cloud_filtered->points[i].z;
    float           y = -cloud_filtered->points[i].x;
    float           z = -cloud_filtered->points[i].y;
    Eigen::Vector3f p(x, y, z);
    Eigen::Vector3f p_w = R * p;
    if (inRange(p)) {
      valid_clouds_[n_valid * 3 + 0] = x;
      valid_clouds_[n_valid * 3 + 1] = y;
      valid_clouds_[n_valid * 3 + 2] = z;
      n_valid++;
      if (n_valid >= 5000) {
        break;
      }
    }
  }
  clock_t t_update_0 = clock();
  std::cout << "number of valid points: " << n_valid << std::endl;
  if (!dsp_map_->update(n_valid, 3, valid_clouds_, pos.x(), pos.y(), pos.z(), t, q.w(), q.x(),
                        q.y(), q.z())) {
    return;
  }
  clock_t t_update_1 = clock();
  double  duration   = static_cast<double>((t_update_1 - t_update_0) * 1000 / CLOCKS_PER_SEC);
  std::cout << "update time: " << duration << "ms" << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vis_mapping_node");
  ros::NodeHandle nh("~");
  int             pool_size_x = 100, pool_size_y = 100, pool_size_z = 60;
  start_pos_ = Eigen::Vector3d(0, 0, 0);

  nh.getParam("init_x", start_pos_(0));
  nh.getParam("init_y", start_pos_(1));
  nh.getParam("init_z", start_pos_(2));
  nh.getParam("vel", vel_);
  nh.getParam("time_step", dt_);
  nh.getParam("pool_size_x", pool_size_x);
  nh.getParam("pool_size_y", pool_size_y);
  nh.getParam("pool_size_z", pool_size_z);
  nh.getParam("is_pose_sub", is_pose_sub_);

  /* initialize map */
  ROS_INFO("init map");
  dsp_map_.reset(new DSPMapStaticV2);
  dsp_map_->initMap(nh);
  dsp_map_->setPredictionVariance(0.05, 0.05);
  dsp_map_->setObservationStdDev(0.05f);
  dsp_map_->setLocalizationStdDev(0.0f);
  dsp_map_->setNewBornParticleNumberofEachPoint(20);
  dsp_map_->setNewBornParticleWeight(0.0001);
  DSPMapStaticV2::setOriginalVoxelFilterResolution(0.15);

  // dsp_map_->initMap(nh);

  ROS_INFO("Start position: (%f, %f, %f)", start_pos_(0), start_pos_(1), start_pos_(2));
  cur_pos_   = start_pos_;
  click_sub_ = nh.subscribe("/move_base_simple/goal", 1, clickCallback);

  cloud_sub_.reset(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "grid_map/cloud", 30));
  if (!is_pose_sub_) { /* use odometry */
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
        nh, "grid_map/odom", 100, ros::TransportHints().tcpNoDelay()));

    sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
        SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
    sync_cloud_odom_->registerCallback(boost::bind(&cloudOdomCallback, _1, _2));
  } else { /* use pose */
    pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(
        nh, "grid_map/pose", 100, ros::TransportHints().tcpNoDelay()));
    sync_cloud_pose_.reset(new message_filters::Synchronizer<SyncPolicyCloudPose>(
        SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_));
    sync_cloud_pose_->registerCallback(boost::bind(&cloudPoseCallback, _1, _2));
  }

  cmd_pub_   = nh.advertise<quadrotor_msgs::PositionCommand>("/pos_command", 1);
  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflated", 1, true);

  ros::Timer move_timer = nh.createTimer(ros::Duration(dt_), moveCallback);
  ros::Timer pub_timer  = nh.createTimer(ros::Duration(0.05), pubCallback);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}