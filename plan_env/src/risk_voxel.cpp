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

  
  dsp_map::MappingParameters mp;

  /* read parameters */
  nh_.param("map/n_risk_map", mp.n_risk_map_, 3);
  nh_.param("map/n_prediction_per_risk", mp.n_prediction_per_risk_map_, 3);
  nh_.param("map/n_particles_max", mp.n_particles_max_, 1000);
  nh_.param("map/n_particles_max_per_voxel", mp.n_particles_max_per_voxel_, 18);
  nh_.param("map/n_particles_max_per_pyramid", mp.n_particles_max_per_pyramid_, 100);

  nh_.param("map/resolution", mp.resolution_, -1.0F);
  nh_.param("map/map_size_x", mp.map_size_x_, -1.0F);
  nh_.param("map/map_size_y", mp.map_size_y_, -1.0F);
  nh_.param("map/map_size_z", mp.map_size_z_, -1.0F);
  nh_.param("map/local_update_range_x", mp.local_update_range_(0), -1.0F);
  nh_.param("map/local_update_range_y", mp.local_update_range_(1), -1.0F);
  nh_.param("map/local_update_range_z", mp.local_update_range_(2), -1.0F);
  nh_.param("map/obstacles_inflation", mp.obstacles_inflation_, -1.0F);

  nh_.param("map/angle_resolution", mp.angle_resolution_, 3);
  nh_.param("map/half_fov_horizontal", mp.half_fov_h_, -1);
  nh_.param("map/half_fov_vertical", mp.half_fov_v_, -1);

  nh_.param("map/visualization_truncate_height", mp.visualization_truncate_height_, -0.1F);
  nh_.param("map/virtual_ceil_height", mp.virtual_ceil_height_, -0.1F);
  nh_.param("map/virtual_ceil_yp", mp.virtual_ceil_yp_, -0.1F);
  nh_.param("map/virtual_ceil_yn", mp.virtual_ceil_yn_, -0.1F);

  nh_.param("map/show_occ_time", mp.show_occ_time_, false);

  nh_.param("map/newborn/particles_number", mp.newborn_particles_per_point_, 20);
  nh_.param("map/newborn/particles_weight", mp.newborn_particles_weight_, 0.0001F);
  nh_.param("map/newborn/objects_weight", mp.newborn_objects_weight_, 0.04F);

  /* standard derivations */
  nh_.param("map/stddev_pos", mp.stddev_pos_predict_, 0.05F); /* prediction variance */
  nh_.param("map/stddev_vel", mp.stddev_vel_predict_, 0.05F); /* prediction variance */
  nh_.param("map/sigma_update", mp.sigma_update_, -1.0F);
  nh_.param("map/sigma_observation", mp.sigma_obsrv_, -1.0F);
  nh_.param("map/sigma_localization", mp.sigma_loc_, -1.0F);

  nh_.param("map/frame_id", mp.frame_id_, string("world"));
  nh_.param("map/local_map_margin", mp.local_map_margin_, 1);
  nh_.param("map/ground_height", mp.ground_height_, 1.0F);

  nh_.param("map/odom_depth_timeout", mp.odom_depth_timeout_, 1.0F);
  nh_.param("map/is_output_csv", mp.is_csv_output_, false);

  nh_.param("map/is_pose_sub", is_pose_sub_, false);
  nh_.param("map/local_update_range_x", local_update_range_x_, 5.0F);
  nh_.param("map/local_update_range_y", local_update_range_y_, 5.0F);
  nh_.param("map/local_update_range_z", local_update_range_z_, 4.0F);
  nh_.param("map/risk_threshold", risk_threshold_, 0.6F);
  nh_.param("map/static_clearance", clearance_, 0.3F);

  ROS_INFO("Init risk voxel map");
  dsp_map_.reset(new dsp_map::DSPMapStaticV2());
  dsp_map_->initMap(mp);
  dsp_map::DSPMapStaticV2::setOriginalVoxelFilterResolution(0.15);

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
  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map/occupancy_inflated", 1, true);
  obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("vis_obstacle", 1, true);
  /* publish point clouds in 20 Hz */
  pub_timer_ = nh.createTimer(ros::Duration(0.05), &RiskVoxel::pubCallback, this);
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
  // std::cout << "number of valid points: " << n_valid << std::endl;
  if (!dsp_map_->update(n_valid, 3, valid_clouds_, pos, q, t)) {
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
  // std::cout << "number of valid points: " << n_valid << std::endl;
  if (!dsp_map_->update(n_valid, 3, valid_clouds_, p, q, t)) {
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
  pcl::PointCloud<pcl::PointXYZ> cloud;
  dsp_map_->getOccupancyMapWithRiskMaps(num_occupied, cloud, &risk_maps_[0][0], risk_threshold_);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp    = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  cloud_pub_.publish(cloud_msg);
  clock_t t2 = clock();
  // std::cout << "num_occupied: " << num_occupied << std::endl;
  // std::cout << "publish time (ms): " << (t2 - t1) * 1000 / (double)CLOCKS_PER_SEC << std::endl;
}

void RiskVoxel::getObstaclePoints(const float &threshold, std::vector<Eigen::Vector3d> &points) {
  int num_occupied = 0;
  dsp_map_->getObstaclePoints(num_occupied, points, threshold, clearance_);

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
  if (risk > 0.5) {
    std::cout << "risk: " << risk << std::endl;
    return 1;
  }
  return 0;
}