/**
 * @file risk_voxel.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-23
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <plan_env/risk_voxel.h>

void RiskVoxel::init(ros::NodeHandle &nh) {
  nh_ = nh;

  /* Parameters */
  float resolution;
  nh_.param("map/resolution", resolution, 0.1F);
  nh_.param("map/is_pose_sub", is_pose_sub_, false);
  nh_.param("map/local_update_range_x", local_update_range_x_, 5.0F);
  nh_.param("map/local_update_range_y", local_update_range_y_, 5.0F);
  nh_.param("map/local_update_range_z", local_update_range_z_, 4.0F);
  nh_.param("map/risk_threshold", risk_threshold_, 0.2F);
  nh_.param("map/static_clearance", clearance_, 0.3F);

  ROS_INFO("Init risk voxel map");
  dsp_map_.reset(new dsp_map::DSPMap());
  dsp_map_->setPredictionVariance(
      0.05, 0.05);  // StdDev for prediction. velocity StdDev, position StdDev, respectively.
  dsp_map_->setObservationStdDev(0.1);  // StdDev for update. position StdDev.
  dsp_map_->setNewBornParticleNumberofEachPoint(
      20);  // Number of new particles generated from one measurement point.
  dsp_map_->setNewBornParticleWeight(0.0001);  // Initial weight of particles.
  dsp_map::DSPMap::setOriginalVoxelFilterResolution(
      resolution);  // Resolution of the voxel filter used for point cloud pre-process.
  dsp_map_->setParticleRecordFlag(
      0, 19.0);  // Set the first parameter to 1 to save particles at a time: e.g. 19.0s. Saving
                 // will take a long time. Don't use it in realtime applications.

  cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_->points.reserve(80000);

  /* subscribers */
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "map/cloud", 30));
  if (!is_pose_sub_) { /* use odometry */
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
        nh, "map/odom", 100, ros::TransportHints().tcpNoDelay()));

    sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
        SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
    sync_cloud_odom_->registerCallback(boost::bind(&RiskVoxel::cloudOdomCallback, this, _1, _2));
  } else { /* use pose */
    pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(
        nh, "map/pose", 100, ros::TransportHints().tcpNoDelay()));
    sync_cloud_pose_.reset(new message_filters::Synchronizer<SyncPolicyCloudPose>(
        SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_));
    sync_cloud_pose_->registerCallback(boost::bind(&RiskVoxel::cloudPoseCallback, this, _1, _2));
  }

  /* publishers */
  cloud_pub_    = nh.advertise<sensor_msgs::PointCloud2>("map/occupancy_inflated", 1, true);
  obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("vis_obstacle", 1, true);
  /* publish point clouds in 20 Hz */
  pub_timer_ = nh.createTimer(ros::Duration(0.10), &RiskVoxel::pubCallback, this);
}

/**
 * @brief filter point clouds, keep points in range
 *
 * @param cloud_in input point cloud in camera frame
 * @param cloud_out output point cloud in world frame
 * @param valid_clouds array of valid points in world frame
 * @param valid_clouds_num number of valid points
 */
void RiskVoxel::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr &      cloud_out,
                                 float *                                    valid_clouds,
                                 int &                                      valid_clouds_num) {
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in);
  sor.setLeafSize(0.2, 0.2, 0.2);
  sor.filter(*cloud_out);

  /* filter points which are too far from the drone */
  for (int i = 0; i < cloud_out->points.size(); i++) {
    float           x = cloud_out->points[i].z;
    float           y = -cloud_out->points[i].x;
    float           z = -cloud_out->points[i].y;
    Eigen::Vector3f p(x, y, z);
    if (isInRange(p)) {
      valid_clouds[valid_clouds_num * 3 + 0] = x;
      valid_clouds[valid_clouds_num * 3 + 1] = y;
      valid_clouds[valid_clouds_num * 3 + 2] = z;
      valid_clouds_num++;
      if (valid_clouds_num >= 5000) {
        break;
      }
    }
  }
}

/**
 * @brief subscribe point cloud and odometry
 *
 * @param cloud_msg point cloud in camera frame
 * @param pose_msg geometry_msgs::PoseStamped
 */
void RiskVoxel::cloudPoseCallback(const sensor_msgs::PointCloud2::ConstPtr &  cloud_msg,
                                  const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
  pose_             = Eigen::Vector3f(pose_msg->pose.position.x, pose_msg->pose.position.y,
                          pose_msg->pose.position.z);
  q_                = Eigen::Quaternionf(pose_msg->pose.orientation.w, pose_msg->pose.orientation.x,
                          pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);
  // Eigen::Matrix3f R = q_.toRotationMatrix();
  double          t = cloud_msg->header.stamp.toSec();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *cloud_in);

  int n_valid = 0;
  filterPointCloud(cloud_in, cloud_filtered, valid_clouds_, n_valid);

  clock_t t_update_0 = clock();
  // std::cout << "number of valid points: " << n_valid << std::endl;
  if (!dsp_map_->update(n_valid, 3, valid_clouds_, pose_.x(), pose_.y(), pose_.z(), t, q_.w(),
                        q_.x(), q_.y(), q_.z())) {
    return;
  }
  clock_t t_update_1 = clock();
  // std::cout << "update time (ms): " << (t_update_1 - t_update_0) * 1000 / CLOCKS_PER_SEC
  //           << std::endl;
}

/**
 * @brief subscribe point cloud and odometry
 *
 * @param cloud_msg point cloud in camera frame
 * @param odom_msg nav_msgs::Odometry
 */
void RiskVoxel::cloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                                  const nav_msgs::Odometry::ConstPtr &      odom_msg) {
  pose_ = Eigen::Vector3f(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                          odom_msg->pose.pose.position.z);
  q_    = Eigen::Quaternionf(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                          odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
  // Eigen::Matrix3f R = q_.toRotationMatrix();
  double          t = cloud_msg->header.stamp.toSec();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *cloud_in);

  int n_valid = 0;
  filterPointCloud(cloud_in, cloud_filtered, valid_clouds_, n_valid);

  clock_t t_update_0 = clock();
  // std::cout << "number of valid points: " << n_valid << std::endl;
  if (!dsp_map_->update(n_valid, 3, valid_clouds_, pose_.x(), pose_.y(), pose_.z(), t, q_.w(),
                        q_.x(), q_.y(), q_.z())) {
    return;
  }
  clock_t t_update_1 = clock();
  // std::cout << "update time(ms): " << (t_update_1 - t_update_0) * 1000 / CLOCKS_PER_SEC
  //           << std::endl;
}

/**
 * @brief publish point clouds in a fixed frequency
 *
 * @param event
 */
void RiskVoxel::pubCallback(const ros::TimerEvent &event) { publishOccMap(); }

void RiskVoxel::publishOccMap() {
  int                            num_occupied = 0;
  clock_t                        t1           = clock();
  cloud_->points.clear();
  cloud_->width  = 0;
  cloud_->height = 0;
  dsp_map_->getOccupancyMapWithFutureStatus(num_occupied, *cloud_, &risk_maps_[0][0],
                                            risk_threshold_);

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translate(pose_);
  pcl::transformPointCloud(*cloud_, *cloud_, transform);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_, cloud_msg);
  cloud_msg.header.stamp    = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  cloud_pub_.publish(cloud_msg);
  clock_t t2 = clock();
  // std::cout << "num_occupied: " << num_occupied << std::endl;
  // std::cout << "publish time (ms): " << (t2 - t1) * 1000 / (double)CLOCKS_PER_SEC << std::endl;
}

void RiskVoxel::getObstaclePoints(std::vector<Eigen::Vector3d> &points) {
  int num_occupied = 0;

  // convert cloud to Eigen::Vector3d
  if (cloud_->points.size() > 0) {
    for (int i = 0; i < cloud_->points.size(); i++) {
      Eigen::Vector3d p;
      p.x() = cloud_->points[i].x;
      p.y() = cloud_->points[i].y;
      p.z() = cloud_->points[i].z;
      points.push_back(p);
    }
  }
  // dsp_map_->getObstaclePoints(num_occupied, points, threshold, clearance_);

  /* Debug: publish these obstacle points */
  // pcl::PointCloud<pcl::PointXYZ> cloud;
  // for (int i = 0; i < points.size(); i++) {
  //   pcl::PointXYZ p;
  //   p.x = points[i](0);
  //   p.y = points[i](1);
  //   p.z = points[i](2);
  //   cloud.push_back(p);
  // }
  // sensor_msgs::PointCloud2 cloud_msg;
  // pcl::toROSMsg(cloud, cloud_msg);
  // cloud_msg.header.stamp    = ros::Time::now();
  // cloud_msg.header.frame_id = "world";
  // obstacle_pub_.publish(cloud_msg);
}

int RiskVoxel::getInflateOccupancy(Eigen::Vector3d pos) {
  int   index;
  float px = static_cast<float>(pos(0));
  float py = static_cast<float>(pos(1));
  float pz = static_cast<float>(pos(2));

  if (!dsp_map_->getPointVoxelsIndexPublic(px, py, pz, index)) {
    return -1;
  }
  float risk = risk_maps_[index][0];
  if (risk > risk_threshold_) {
    return 1;
  }
  return 0;
}