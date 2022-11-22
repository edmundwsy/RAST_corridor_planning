/**
 * @file risk_voxel.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-23
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef RISK_VOXEL_H
#define RISK_VOXEL_H

#include <plan_env/dsp_dynamic.h>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>

class RiskVoxel {
 protected:
  /* ROS Utilities */
  ros::NodeHandle nh_;
  ros::Subscriber click_sub_;
  ros::Publisher  cloud_pub_;
  ros::Publisher  risk_pub_;
  ros::Publisher  obstacle_pub_; /* Debug */
  ros::Timer      pub_timer_;

  /* Data */
  dsp_map::DSPMap::Ptr        dsp_map_;
  Eigen::Vector3f    pose_;
  Eigen::Quaternionf q_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

  /* Parameters */
  float risk_maps_[VOXEL_NUM][PREDICTION_TIMES];
  float valid_clouds_[5000 * 3];
  float local_update_range_x_;
  float local_update_range_y_;
  float local_update_range_z_;
  float risk_threshold_;
  float clearance_;

  /* Message filters */
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

  /* Utilities */
  inline bool isInRange(const Eigen::Vector3f &p);

 public:
  RiskVoxel() {}
  ~RiskVoxel() {}

  void init(ros::NodeHandle &nh);
  void publishOccMap();

  inline void getMapCenter(Eigen::Vector3f &center) { center = pose_; }

  void pubCallback(const ros::TimerEvent &event);
  void cloudPoseCallback(const sensor_msgs::PointCloud2::ConstPtr &  cloud_msg,
                         const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
  void cloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                         const nav_msgs::Odometry::ConstPtr &      odom_msg);

  void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &      cloud_out,
                        float *                                    valid_clouds,
                        int &                                      valid_clouds_num);
  void getObstaclePoints(std::vector<Eigen::Vector3d> &points);
  int  getInflateOccupancy(Eigen::Vector3d pos);

  typedef std::shared_ptr<RiskVoxel> Ptr;
};

/* ====================== definition of inline function ====================== */

inline bool RiskVoxel::isInRange(const Eigen::Vector3f &p) {
  return p.x() > -local_update_range_x_ && p.x() < local_update_range_x_ &&
         p.y() > -local_update_range_y_ && p.y() < local_update_range_y_ &&
         p.z() > -local_update_range_z_ && p.z() < local_update_range_z_;
}

#endif /* RISK_VOXEL_H */