/**
 * @file fake_dsp_map.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief subscribe ground truth map and velocity,
 *       generate fake map prediction
 * @version 1.0
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef FAKE_DSP_MAP_H
#define FAKE_DSP_MAP_H

#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <plan_env/risk_voxel.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

struct Cylinder {
  double x;   // x coordinate
  double y;   // y coordinate
  double w;   // width
  double h;   // height
  double vx;  // velocity x
  double vy;  // velocity y
};

/**
 * @brief Class for generating fake map prediction, which is inherited from RiskVoxel
 */
class FakeRiskVoxel : public RiskVoxel {
 public:
  FakeRiskVoxel() {}
  ~FakeRiskVoxel() {}
  void init(ros::NodeHandle &nh);
  void groundTruthMapCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
  void groundTruthStateCallback(const visualization_msgs::MarkerArray::ConstPtr &state_msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
  void pubCallback(const ros::TimerEvent &event);
  typedef std::shared_ptr<FakeRiskVoxel> Ptr;

  void getObstaclePoints(std::vector<Eigen::Vector3d> &points);
  int  getInflateOccupancy(Eigen::Vector3d pos);

 private:
  /* Inline helper functions */
  inline bool            isInRange(const Eigen::Vector3f &p);
  inline int             getVoxelIndex(const Eigen::Vector3f &pos);
  inline Eigen::Vector3f getVoxelPosition(int index);

 private:
  float resolution_;
  /* ROS Utilities */
  ros::Subscriber       gt_map_sub_;    // ground truth local map
  ros::Subscriber       gt_state_sub_;  // ground truth velocity
  ros::Subscriber       odom_sub_;
  ros::Subscriber       pose_sub_;
  std::vector<Cylinder> gt_cylinders_;
};

inline bool FakeRiskVoxel::isInRange(const Eigen::Vector3f &p) {
  return p.x() - pose_.x() < local_update_range_x_ && p.x() - pose_.x() > -local_update_range_x_ &&
         p.y() - pose_.y() < local_update_range_y_ && p.y() - pose_.y() > -local_update_range_y_ &&
         p.z() - pose_.z() < local_update_range_z_ && p.z() - pose_.z() > -local_update_range_z_;
}

inline int FakeRiskVoxel::getVoxelIndex(const Eigen::Vector3f &pos) {
  int x = (pos[0] - pose_[0] + local_update_range_x_) / resolution_;
  int y = (pos[1] - pose_[1] + local_update_range_y_) / resolution_;
  int z = (pos[2] - pose_[2] + local_update_range_z_) / resolution_;
  return z * MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM + y * MAP_LENGTH_VOXEL_NUM + x;
}

inline Eigen::Vector3f FakeRiskVoxel::getVoxelPosition(int index) {
  int x = index % MAP_LENGTH_VOXEL_NUM;
  int y = (index / MAP_LENGTH_VOXEL_NUM) % MAP_WIDTH_VOXEL_NUM;
  int z = index / (MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM);
  return Eigen::Vector3f(x * resolution_ + pose_[0] - local_update_range_x_,
                         y * resolution_ + pose_[1] - local_update_range_y_,
                         z * resolution_ + pose_[2] - local_update_range_z_);
}

#endif  // FAKE_DSP_MAP_H