/**
 * @file dsp_map_node.cpp
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
ros::Publisher  cloud_pub_;

DSPMapStaticV2::Ptr dsp_map_;
float               risk_maps_[VOXEL_NUM][3];
float               valid_clouds_[5000 * 3];
float               local_update_range_x_;
float               local_update_range_y_;
float               local_update_range_z_;

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
bool inRange(const Eigen::Vector3f &p) {
  return p.x() > -local_update_range_x_ && p.x() < local_update_range_x_ &&
         p.y() > -local_update_range_y_ && p.y() < local_update_range_y_ &&
         p.z() > -local_update_range_z_ && p.z() < local_update_range_z_;
}

/**
 * @brief filter point clouds, keep points in range
 *
 * @param cloud_in input point cloud in camera frame
 * @param cloud_out output point cloud in world frame
 * @param valid_clouds array of valid points in world frame
 * @param valid_clouds_num number of valid points
 */
void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
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
    if (inRange(p)) {
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
 * @param odom_msg nav_msgs::Odometry
 */
void cloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                       const nav_msgs::Odometry::ConstPtr &      odom_msg) {
  Eigen::Vector3f    p(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                    odom_msg->pose.pose.position.z);
  Eigen::Quaternionf q(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                       odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
  Eigen::Matrix3f    R = q.toRotationMatrix();
  double             t = cloud_msg->header.stamp.toSec();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *cloud_in);

  int n_valid = 0;
  filterPointCloud(cloud_in, cloud_filtered, valid_clouds_, n_valid);

  clock_t t_update_0 = clock();
  std::cout << "number of valid points: " << n_valid << std::endl;
  if (!dsp_map_->update(n_valid, 3, valid_clouds_, p, q, t)) {
    return;
  }
  clock_t t_update_1 = clock();
  std::cout << "update time(ms): " << (t_update_1 - t_update_0) * 1000 / CLOCKS_PER_SEC
            << std::endl;
}

/**
 * @brief subscribe point cloud and odometry
 *
 * @param cloud_msg point cloud in camera frame
 * @param pose_msg geometry_msgs::PoseStamped
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

  int n_valid = 0;
  filterPointCloud(cloud_in, cloud_filtered, valid_clouds_, n_valid);

  clock_t t_update_0 = clock();
  std::cout << "number of valid points: " << n_valid << std::endl;
  if (!dsp_map_->update(n_valid, 3, valid_clouds_, pos, q, t)) {
    return;
  }
  clock_t t_update_1 = clock();
  std::cout << "update time (ms): " << (t_update_1 - t_update_0) * 1000 / CLOCKS_PER_SEC
            << std::endl;
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
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp    = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  cloud_pub_.publish(cloud_msg);
  clock_t t2 = clock();
  std::cout << "num_occupied: " << num_occupied << std::endl;
  std::cout << "publish time (ms): " << (t2 - t1) * 1000 / (double)CLOCKS_PER_SEC << std::endl;
}

/**
 * @brief publish point clouds in a fixed frequency
 *
 * @param event
 */
void pubCallback(const ros::TimerEvent &event) { publishMap(); }

int main(int argc, char **argv) {
  ros::init(argc, argv, "dsp_map_node");
  ros::NodeHandle nh("~");

  nh.param("is_pose_sub", is_pose_sub_, false);
  nh.param("grid_map/local_update_range_x", local_update_range_x_, 5.0F);
  nh.param("grid_map/local_update_range_y", local_update_range_y_, 5.0F);
  nh.param("grid_map/local_update_range_z", local_update_range_z_, 4.0F);

  /* initialize map */
  ROS_INFO("init DSP map");
  dsp_map_.reset(new DSPMapStaticV2);
  dsp_map_->initMap(nh);
  // dsp_map_->setPredictionVariance(0.05, 0.05);
  // dsp_map_->setObservationStdDev(0.05f);
  // dsp_map_->setLocalizationStdDev(0.0f);
  // dsp_map_->setNewBornParticleNumberofEachPoint(20);
  // dsp_map_->setNewBornParticleWeight(0.0001);
  DSPMapStaticV2::setOriginalVoxelFilterResolution(
      0.15);  // TODO(user): it cannot be set in run time

  // dsp_map_->initMap(nh);

  /* subscribers */
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

  /* publishers */
  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflated", 1, true);

  /* publish point clouds in 20 Hz */
  ros::Timer pub_timer = nh.createTimer(ros::Duration(0.05), pubCallback);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}