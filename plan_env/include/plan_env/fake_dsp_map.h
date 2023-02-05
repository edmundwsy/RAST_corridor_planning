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
#include <traj_coordinator/mader.hpp>

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
// class FakeRiskVoxel {
class FakeRiskVoxel : public RiskVoxel {
 public:
  FakeRiskVoxel() {}
  ~FakeRiskVoxel() {}
  void init(ros::NodeHandle &nh);
  void setCoordinator(MADER::Ptr ptr) {
    is_multi_agents_ = true;
    coordinator_     = ptr;
  }
  void groundTruthMapCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
  void groundTruthStateCallback(const visualization_msgs::MarkerArray::ConstPtr &state_msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
  void pubCallback(const ros::TimerEvent &event);

  void addObstacles(const std::vector<Eigen::Vector3d> &centers,
                    const Eigen::Vector3d              &size,
                    int                                 t_index);
  void addObstacles(const std::vector<Eigen::Vector3d> &centers,
                    const Eigen::Vector3d              &size,
                    const ros::Time                    &t);
  void getObstaclePoints(std::vector<Eigen::Vector3d> &points);
  void getObstaclePoints(std::vector<Eigen::Vector3d> &points, double t_start, double t_end);
  void getObstaclePoints(std::vector<Eigen::Vector3d> &points,
                         double                        t_start,
                         double                        t_end,
                         const Eigen::Vector3d        &lc,
                         const Eigen::Vector3d        &hc);
  int  getInflateOccupancy(const Eigen::Vector3d pos);
  int  getInflateOccupancy(const Eigen::Vector3d pos, int t);
  int  getInflateOccupancy(const Eigen::Vector3d pos, double t);

  inline void setMapCenter(const Eigen::Vector3f &center) { pose_ = center; }
  inline void setQuaternion(const Eigen::Quaternionf &q) { q_ = q; }
  typedef std::shared_ptr<FakeRiskVoxel> Ptr;

 private:
  /* Inline helper functions */
  inline Eigen::Vector3i getVoxelRelIndex(const Eigen::Vector3f &pos);
  inline int             getVoxelIndex(const Eigen::Vector3f &pos);
  inline bool            isInRange(const Eigen::Vector3f &p);
  inline Eigen::Vector3f getVoxelPosition(int index);
  inline Eigen::Vector3f getVoxelRelPosition(int index);

 private:
  bool            is_publish_spatio_temporal_map_;
  bool            is_multi_agents_ = false;
  float           resolution_;
  float           time_resolution_;
  ros::Time       last_update_time_;  // last map update time (start time of the risk map)
  ros::Subscriber gt_map_sub_;        // ground truth local map
  ros::Subscriber gt_state_sub_;      // ground truth velocity
  ros::Subscriber odom_sub_;
  ros::Subscriber pose_sub_;
  /* Data Variables */
  std::vector<Cylinder> gt_cylinders_;
  MADER::Ptr            coordinator_;  // TODO(01.10): move this to risk_voxel
};

/**
 * @brief if the point is in the map range
 * @param p point in the map frame
 * @return true: in range
 */
inline bool FakeRiskVoxel::isInRange(const Eigen::Vector3f &p) {
  return (fabs(p.x()) < local_update_range_x_ && fabs(p.y()) < local_update_range_y_ &&
          fabs(p.z()) < local_update_range_z_);
}
/**
 * @brief get the index of the voxel in the local frame
 * @param pos point in map frame
 * @return index
 */
inline int FakeRiskVoxel::getVoxelIndex(const Eigen::Vector3f &pos) {
  int x = (pos[0] + local_update_range_x_) / resolution_;
  int y = (pos[1] + local_update_range_y_) / resolution_;
  int z = (pos[2] + local_update_range_z_) / resolution_;
  return z * MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM + y * MAP_LENGTH_VOXEL_NUM + x;
}

/**
 * @brief get voxel index in local frame
 *
 * @param pos index in global frame
 */
inline Eigen::Vector3i FakeRiskVoxel::getVoxelRelIndex(const Eigen::Vector3f &pos) {
  int x = (pos[0] + local_update_range_x_) / resolution_;
  int y = (pos[1] + local_update_range_y_) / resolution_;
  int z = (pos[2] + local_update_range_z_) / resolution_;
  return Eigen::Vector3i(x, y, z);
}

/**
 * @brief
 * @param index
 * @return position of the voxel in the world frame
 */
inline Eigen::Vector3f FakeRiskVoxel::getVoxelPosition(int index) {
  int x = index % MAP_LENGTH_VOXEL_NUM;
  int y = (index / MAP_LENGTH_VOXEL_NUM) % MAP_WIDTH_VOXEL_NUM;
  int z = index / (MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM);
  return Eigen::Vector3f(x * resolution_ - local_update_range_x_,
                         y * resolution_ - local_update_range_y_,
                         z * resolution_ - local_update_range_z_) +
         pose_;
}

inline Eigen::Vector3f FakeRiskVoxel::getVoxelRelPosition(int index) {
  int x = index % MAP_LENGTH_VOXEL_NUM;
  int y = (index / MAP_LENGTH_VOXEL_NUM) % MAP_WIDTH_VOXEL_NUM;
  int z = index / (MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM);
  return Eigen::Vector3f(x * resolution_ - local_update_range_x_,
                         y * resolution_ - local_update_range_y_,
                         z * resolution_ - local_update_range_z_);
}

#endif  // FAKE_DSP_MAP_H
