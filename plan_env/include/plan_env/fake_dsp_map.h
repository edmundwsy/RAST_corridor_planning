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

/* ----- Definition of these functions ----- */
/**
 * @brief initialize the fake risk voxel, define subscriptions and publications
 *
 * @param nh
 */
void FakeRiskVoxel::init(ros::NodeHandle &nh) {
  nh_ = nh;

  /* parameters */
  nh_.param("map/is_pose_sub", is_pose_sub_, false);
  nh_.param("map/local_update_range_x", local_update_range_x_, 5.0F);
  nh_.param("map/local_update_range_y", local_update_range_y_, 5.0F);
  nh_.param("map/local_update_range_z", local_update_range_z_, 4.0F);
  nh_.param("map/risk_threshold", risk_threshold_, 0.2F);
  nh_.param("map/static_clearance", clearance_, 0.3F);
  resolution_           = 0.1F;
  local_update_range_x_ = MAP_LENGTH_VOXEL_NUM / 2 * resolution_;
  local_update_range_y_ = MAP_WIDTH_VOXEL_NUM / 2 * resolution_;
  local_update_range_z_ = MAP_HEIGHT_VOXEL_NUM / 2 * resolution_;

  ROS_INFO("Init fake risk voxel map");

  cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_->points.reserve(80000);

  /* subscribers */
  gt_map_sub_   = nh_.subscribe("map/cloud", 1, &FakeRiskVoxel::groundTruthMapCallback, this);
  gt_state_sub_ = nh_.subscribe("map/state", 1, &FakeRiskVoxel::groundTruthStateCallback, this);
  odom_sub_     = nh_.subscribe("map/odom", 1, &FakeRiskVoxel::odomCallback, this);
  pose_sub_     = nh_.subscribe("map/pose", 1, &FakeRiskVoxel::poseCallback, this);

  /* publishers */
  cloud_pub_    = nh.advertise<sensor_msgs::PointCloud2>("map/occupancy_inflated", 1, true);
  obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("vis_obstacle", 1, true);
  /* publish point clouds in 10 Hz */
  pub_timer_ = nh.createTimer(ros::Duration(0.10), &FakeRiskVoxel::pubCallback, this);
}

/**
 * @brief
 *
 * @param pose_msg
 */
void FakeRiskVoxel::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
  pose_ = Eigen::Vector3f(pose_msg->pose.position.x, pose_msg->pose.position.y,
                          pose_msg->pose.position.z);
  q_    = Eigen::Quaternionf(pose_msg->pose.orientation.w, pose_msg->pose.orientation.x,
                          pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);
}

/**
 * @brief
 *
 * @param odom_msg
 */
void FakeRiskVoxel::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  pose_ = Eigen::Vector3f(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                          odom_msg->pose.pose.position.z);
  q_    = Eigen::Quaternionf(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                          odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
}

/**
 * @brief
 *
 * @param cloud_msg
 */
void FakeRiskVoxel::groundTruthMapCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
  pcl::fromROSMsg(*cloud_msg, *cloud_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  /* Remove points out of range */
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(cloud_);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(pose_[0] - local_update_range_x_, pose_[0] + local_update_range_x_);
  pass_x.filter(*cloud_filtered);

  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(cloud_filtered);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(pose_[1] - local_update_range_y_, pose_[1] + local_update_range_y_);
  pass_y.filter(*cloud_filtered);

  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(cloud_filtered);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(pose_[2] - local_update_range_z_, pose_[2] + local_update_range_z_);
  pass_z.filter(*cloud_filtered);

  /* clear risk_maps_ */
  for (int i = 0; i < VOXEL_NUM; i++) {
    for (int j = 0; j < PREDICTION_TIMES; j++) {
      risk_maps_[i][j] = 0.0F;
    }
  }

  for (auto &point : cloud_filtered->points) {
    Eigen::Vector3f pt = Eigen::Vector3f(point.x, point.y, point.z);

    if (isInRange(pt)) {
      int idx = getVoxelIndex(pt);

      risk_maps_[idx][0] = 1.0F;
    }

    for (auto &cyl : gt_cylinders_) {
      Eigen::Vector3f pt_cyl       = Eigen::Vector3f(cyl.x, cyl.y, pt.z());
      Eigen::Vector3f pt_cyl_to_pt = pt - pt_cyl;
      Eigen::Vector3f vel          = Eigen::Vector3f(cyl.vx, cyl.vy, 0.0F);
      float           dist         = pt_cyl_to_pt.norm();
      if (dist < cyl.w) { /* point is assigned to cylinder */
        for (int k = 1; k < PREDICTION_TIMES; k++) {
          Eigen::Vector3f pt_pred = pt + vel * 0.2F * k;
          if (isInRange(pt_pred)) {
            int idx            = getVoxelIndex(pt_pred);
            risk_maps_[idx][k] = 1.0F;
          }
        }
        break;
      }
    }
  }
}

/**
 * @brief
 * @param state_msg
 */
void FakeRiskVoxel::groundTruthStateCallback(
    const visualization_msgs::MarkerArray::ConstPtr &state_msg) {
  std::cout << "ground truth state callback" << std::endl;
  int n = state_msg->markers.size();
  std::cout << "number of markers: " << n << std::endl;
  gt_cylinders_.clear();
  gt_cylinders_.resize(n);

  for (auto &mk : state_msg->markers) {
    gt_cylinders_[mk.id].x  = mk.points[0].x;
    gt_cylinders_[mk.id].y  = mk.points[0].y;
    gt_cylinders_[mk.id].w  = mk.scale.x;
    gt_cylinders_[mk.id].h  = mk.points[0].z;
    gt_cylinders_[mk.id].vx = mk.points[1].x - mk.points[0].x;
    gt_cylinders_[mk.id].vy = mk.points[1].y - mk.points[0].y;
  }
}

int FakeRiskVoxel::getInflateOccupancy(Eigen::Vector3d pos) {
  // TODO
  printf("getInflateOccupancy");
}

void FakeRiskVoxel::pubCallback(const ros::TimerEvent &event) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.reserve(VOXEL_NUM);
  for (int i = 0; i < VOXEL_NUM; i++) {
    if (risk_maps_[i][2] > clearance_) {
      Eigen::Vector3f pt = getVoxelPosition(i);
      pcl::PointXYZ   p;
      p.x = pt[0];
      p.y = pt[1];
      p.z = pt[2];
      cloud->points.push_back(p);
    }
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp    = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  cloud_pub_.publish(cloud_msg);
}

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