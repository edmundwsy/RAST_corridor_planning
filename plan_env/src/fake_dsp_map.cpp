/**
 * @file fake_dsp_map.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-11-22
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <plan_env/fake_dsp_map.h>

/* ----- Definition of these functions ----- */
/**
 * @brief initialize the fake risk voxel, define subscriptions and publications
 *
 * @param nh
 */
void FakeRiskVoxel::init(ros::NodeHandle &nh) {
  nh_ = nh;

  /* parameters */
  nh_.param("map/booleans/sub_pose", is_pose_sub_, false);
  nh_.param("map/local_update_range_x", local_update_range_x_, 5.0F);
  nh_.param("map/local_update_range_y", local_update_range_y_, 5.0F);
  nh_.param("map/local_update_range_z", local_update_range_z_, 4.0F);
  nh_.param("map/risk_threshold", risk_threshold_, 0.2F);
  nh_.param("map/clearance", clearance_, 0.3F);
  nh_.param("map/time_resolution", time_resolution_, 0.2F);
  nh_.param("map/booleans/pub_spatio_temporal", is_publish_spatio_temporal_map_, false);

  resolution_           = 0.1F;
  local_update_range_x_ = MAP_LENGTH_VOXEL_NUM / 2 * resolution_;
  local_update_range_y_ = MAP_WIDTH_VOXEL_NUM / 2 * resolution_;
  local_update_range_z_ = MAP_HEIGHT_VOXEL_NUM / 2 * resolution_;
  ROS_INFO("[MAP] Local update range: %f, %f, %f", local_update_range_x_, local_update_range_y_,
           local_update_range_z_);
  ROS_INFO("[MAP] Init fake risk voxel map");

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

  /* initialize odometry */
  pose_ = Eigen::Vector3f::Zero();
  q_    = Eigen::Quaternionf::Identity();
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
  /* ground truth starting time */
  last_update_time_ = ros::Time::now();
  ros::Time tic     = last_update_time_;

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

  /* update map fuse perception and ground truth information */
  // for (auto &point : cloud_filtered->points) {
  //   Eigen::Vector3f pt = Eigen::Vector3f(point.x, point.y, point.z) - pose_;
  //
  //   /* add observed points to the first layer of map */
  //   if (isInRange(pt)) {
  //     int idx = getVoxelIndex(pt);
  //
  //     risk_maps_[idx][0] = 1.0F;
  //   }
  //
  //   /* add ground truth obstacles to other layers of map */
  //   for (auto &cyl : gt_cylinders_) {
  //     Eigen::Vector3f pt_cyl       = Eigen::Vector3f(cyl.x, cyl.y, pt.z() + pose_.z());
  //     Eigen::Vector3f pt_cyl_to_pt = pt + pose_ - pt_cyl;
  //
  //     /* get observed point's distance to ground truth position, move these points to desired
  //      * position in the future maps*/
  //     float dist = pt_cyl_to_pt.norm();
  //     if (dist < cyl.w) { /* point is assigned to cylinder */
  //       Eigen::Vector3f vel = Eigen::Vector3f(cyl.vx, cyl.vy, 0.0F);
  //       for (int k = 1; k < PREDICTION_TIMES; k++) {
  //         Eigen::Vector3f pt_pred = pt + vel * time_resolution_ * k;
  //         if (isInRange(pt_pred)) {
  //           int idx            = getVoxelIndex(pt_pred);
  //           risk_maps_[idx][k] = 1.0F;
  //         }
  //       }
  //       break;
  //     }
  //   }
  // }

  // ros::Time t0 = ros::Time::now();
  /* add points to map and inflation */
  for (auto &points : cloud_filtered->points) {
    Eigen::Vector3f pt = Eigen::Vector3f(points.x, points.y, points.z) - pose_;
    if (isInRange(pt)) {
      risk_maps_[getVoxelIndex(pt)][0] = 1.0F;
    }
  }
  // ros::Time t1 = ros::Time::now();
  // ROS_INFO("add points to map time: %f", (t1 - t0).toSec());

  /* create convolution kernel */
  int                          inf_steps = (2 * clearance_) / VOXEL_RESOLUTION + 1;
  std::vector<Eigen::Vector3f> inf_pts(std::pow(inf_steps, 3));

  int n = 0;
  for (float xx = -clearance_; xx <= clearance_; xx += VOXEL_RESOLUTION) {
    for (float yy = -clearance_; yy <= clearance_; yy += VOXEL_RESOLUTION) {
      for (float zz = -clearance_; zz <= clearance_; zz += VOXEL_RESOLUTION) {
        inf_pts[n++] = Eigen::Vector3f(xx, yy, zz);
      }
    }
  }
  // ros::Time t2 = ros::Time::now();
  // ROS_INFO("convolution time: %f", (t2 - t1).toSec());

  std::vector<int> obs_idx_list(VOXEL_NUM);
  for (int i = 0; i < VOXEL_NUM; i++) {
    if (risk_maps_[i][0] > risk_threshold_) {
      obs_idx_list.push_back(i);
    }
  }

  for (auto &i : obs_idx_list) {
    /* inflate points */
    Eigen::Vector3f pt_obstacle = getVoxelRelPosition(i);
    for (auto &pt : inf_pts) {
      Eigen::Vector3f p   = pt + pt_obstacle;
      int             idx = getVoxelIndex(p);
      if (!isInRange(p) || idx > VOXEL_NUM) {
        continue;
      }
      // idx     = idx > VOXEL_NUM ? VOXEL_NUM - 1 : idx;
      /*  TODO:temporal code to prevent overflow
      REASON: pose_ update while inflation after range check
      */
      risk_maps_[idx][0] = 1.0F;
    }
  }
  // ros::Time t3 = ros::Time::now();
  // ROS_INFO("inflate time: %f", (t3 - t2).toSec());

  /* read ground truth and construct future maps */
  obs_idx_list.clear();
  for (int i = 0; i < VOXEL_NUM; i++) {
    if (risk_maps_[i][0] > risk_threshold_) {
      obs_idx_list.push_back(i);
    }
  }

  for (auto &i : obs_idx_list) {
    Eigen::Vector3f pt        = getVoxelPosition(i);
    int             obs_count = 0;
    for (auto &cyl : gt_cylinders_) {
      Eigen::Vector3f pt_cyl = Eigen::Vector3f(cyl.x, cyl.y, pt.z());
      if (!isInRange(pt_cyl - pose_)) continue;
      float dist = (pt - pt_cyl).norm();
      if (dist <= cyl.w + clearance_) {
        Eigen::Vector3f vel = Eigen::Vector3f(cyl.vx, cyl.vy, 0.0F);
        for (int k = 1; k < PREDICTION_TIMES; k++) {
          Eigen::Vector3f pt_pred = pt + vel * time_resolution_ * k - pose_;
          if (isInRange(pt_pred)) {
            risk_maps_[getVoxelIndex(pt_pred)][k] = 1.0F;
          }
        }
        obs_count++;
        if (obs_count > 4) break;
      }
    }
  }
  // ros::Time t4 = ros::Time::now();
  // ROS_INFO("construct future map time: %f", (t4 - t3).toSec());

  /* add other agents to the map */
  tic = ros::Time::now();
  if (is_multi_agents_) {
    Eigen::Vector3d robot_size = coordinator_->getAgentsSize();
    int             n          = coordinator_->getNumAgents();
    for (int idx = 0; idx < PREDICTION_TIMES; idx++) {
      std::vector<Eigen::Vector3d> waypoints;
      for (int i = 0; i < n; i++) {
        double t = last_update_time_.toSec() + time_resolution_ * idx;
        /* query coordinator to get the future waypoints from broadcast trajectory */
        coordinator_->getWaypoints(waypoints, i, t);
        /* Get waypoints at the beginning of each time step, and modify map
         * accordingly */
      }
      /* Transfer to local frame */
      for (auto &wp : waypoints) {
        wp = wp - pose_.cast<double>();
      }
      addObstacles(waypoints, robot_size, idx);
    }
    ros::Time toc = ros::Time::now();
    std::cout << "adding obstacles takes: " << (toc - tic).toSec() * 1000 << "ms" << std::endl;
  }
}

/**
 * @brief
 * @param state_msg
 */
void FakeRiskVoxel::groundTruthStateCallback(
    const visualization_msgs::MarkerArray::ConstPtr &state_msg) {
  int n = state_msg->markers.size();
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

/**
 * @brief check if the point is in obstacle
 * @param pos: point position in world frame
 * @return -1: out of range, 0: not in obstacle, 1: in obstacle
 */
int FakeRiskVoxel::getInflateOccupancy(const Eigen::Vector3d pos) {
  Eigen::Vector3f pf  = pos.cast<float>() - pose_;  // point in the local frame
  int             idx = getVoxelIndex(pf);
  // std::cout << "pf: " << pf.transpose() << "\t idx: " << idx % MAP_LENGTH_VOXEL_NUM <<
  // "\trange"
  //           << this->isInRange(pf) << "\trisk:" << risk_maps_[idx][0] << std::endl;
  if (!this->isInRange(pf)) return -1;
  if (risk_maps_[idx][0] > risk_threshold_) return 1;
  return 0;
}

/**
 * @brief
 * @param pos in world frame
 * @param t : int
 * @return
 */
int FakeRiskVoxel::getInflateOccupancy(const Eigen::Vector3d pos, int t) {
  Eigen::Vector3f pf  = pos.cast<float>() - pose_;  // point in the local frame
  int             idx = getVoxelIndex(pf);
  // std::cout << "pf: " << pf.transpose() << "\t idx: " << idx % MAP_LENGTH_VOXEL_NUM <<
  // "\trange"
  // //           << this->isInRange(pf) << "\trisk:" << risk_maps_[idx][0] << std::endl;
  if (!this->isInRange(pf)) return -1;
  if (t > PREDICTION_TIMES) return -1;
  if (risk_maps_[idx][t] > risk_threshold_) return 1;
  return 0;
}

/**
 * @brief
 * @param pos in world frame
 * @param t : double FIXME: TIME MISMATCH
 * @return
 */
int FakeRiskVoxel::getInflateOccupancy(const Eigen::Vector3d pos, double t) {
  int tc = ceil(t / time_resolution_);
  tc     = tc > PREDICTION_TIMES ? PREDICTION_TIMES : tc;
  int tf = floor(t / time_resolution_);
  tf     = tf > PREDICTION_TIMES ? PREDICTION_TIMES : tf;

  Eigen::Vector3f pf  = pos.cast<float>() - pose_;  // point in the local frame
  int             idx = getVoxelIndex(pf);
  // std::cout << "pf: " << pf.transpose() << " " << pos.transpose() << "\t ti: " << t << " " << ti
  //           << "\trange" << this->isInRange(pf) << "\trisk:" << risk_maps_[idx][ti] << std::endl;
  if (!this->isInRange(pf)) return -1;
  if (risk_maps_[idx][tc] > risk_threshold_) return 1;
  if (risk_maps_[idx][tf] > risk_threshold_) return 1;
  return 0;
}

/**
 *@brief publish the map in a fixed frequency **@param event * /
 **/
void FakeRiskVoxel::pubCallback(const ros::TimerEvent &event) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.reserve(VOXEL_NUM);

  if (is_publish_spatio_temporal_map_) {  /* publish spatio-temporal map */
    for (int i = 0; i < VOXEL_NUM; i++) { /* publish x-y-t map */
      Eigen::Vector3f pt = getVoxelPosition(i);
      pcl::PointXYZ   p;
      for (int j = 0; j < PREDICTION_TIMES; j++) {
        if (risk_maps_[i][j] > risk_threshold_) {
          p.x = pt[0];
          p.y = pt[1];
          p.z = j * time_resolution_;
          cloud->points.push_back(p);
        }
      }
    }
  } else { /* publish x-y-z map */
    for (int i = 0; i < VOXEL_NUM; i++) {
      Eigen::Vector3f pt = getVoxelPosition(i);
      pcl::PointXYZ   p;
      if (risk_maps_[i][0] > risk_threshold_) {
        p.x = pt[0];
        p.y = pt[1];
        p.z = pt[2];
        cloud->points.push_back(p);
      }
    }
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp    = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  cloud_pub_.publish(cloud_msg);
}

/**
 * @brief return obstacle points in the world frame
 *
 * @param points points in world frame
 */
void FakeRiskVoxel::getObstaclePoints(std::vector<Eigen::Vector3d> &points) {
  points.clear();
  for (int i = 0; i < VOXEL_NUM; i++) {
    if (risk_maps_[i][0] > risk_threshold_) {
      Eigen::Vector3f pt = getVoxelPosition(i);
      points.push_back(pt.cast<double>());
    }
  }
}

void FakeRiskVoxel::getObstaclePoints(std::vector<Eigen::Vector3d> &points,
                                      double                        t_start,
                                      double                        t_end) {
  points.clear();
  int idx_start = floor(t_start / time_resolution_);
  int idx_end   = ceil(t_end / time_resolution_);
  for (int i = 0; i < VOXEL_NUM; i++) {
    for (int j = idx_start; j < idx_end; j++) {
      if (risk_maps_[i][j] > risk_threshold_) {
        Eigen::Vector3f pt = getVoxelPosition(i);
        points.push_back(pt.cast<double>());
        break;
      }
    }
  }
}

/**
 * @brief get the obstacle points in the time interval and the cube
 * @param points : points in world frame
 * @param t_start : global start time
 * @param t_end: global end time
 * @param t_step: time step
 */
void FakeRiskVoxel::getObstaclePoints(std::vector<Eigen::Vector3d> &points,
                                      double                        t_start,
                                      double                        t_end,
                                      const Eigen::Vector3d        &lower_corner,
                                      const Eigen::Vector3d        &higher_corner) {
  double t0 = last_update_time_.toSec();

  int idx_start = floor((t_start - t0) / time_resolution_);
  int idx_end   = ceil((t_end - t0) / time_resolution_);
  std::cout << "idx_start" << idx_start << "idx_end" << idx_end << std::endl;

  int lx = (lower_corner[0] - pose_.x() + local_update_range_x_) / resolution_;
  int ly = (lower_corner[1] - pose_.y() + local_update_range_y_) / resolution_;
  int lz = (lower_corner[2] - pose_.z() + local_update_range_z_) / resolution_;
  int hx = (higher_corner[0] - pose_.x() + local_update_range_x_) / resolution_;
  int hy = (higher_corner[1] - pose_.y() + local_update_range_y_) / resolution_;
  int hz = (higher_corner[2] - pose_.z() + local_update_range_z_) / resolution_;

  hx = std::min(hx, MAP_LENGTH_VOXEL_NUM - 1);
  hy = std::min(hy, MAP_WIDTH_VOXEL_NUM - 1);
  hz = std::min(hz, MAP_HEIGHT_VOXEL_NUM - 1);
  lx = std::max(lx, 0);
  ly = std::max(ly, 0);
  lz = std::max(lz, 0);

  for (int z = lz; z <= hz; z++) {
    for (int y = ly; y <= hy; y++) {
      for (int x = lx; x <= hx; x++) {
        int i = x + y * MAP_LENGTH_VOXEL_NUM + z * MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM;
        for (int j = idx_start; j < idx_end; j++) {
          if (risk_maps_[i][j] > risk_threshold_) {
            Eigen::Vector3f pt = getVoxelPosition(i);
            points.push_back(pt.cast<double>());
          }
        }
      }
    }
  }
}

/**
 * @brief add obstacles to the risk map at the initial time
 * @param points: given obstacle points in the map frame
 * @param size: the size of the obstacle (axis-aligned bounding box)
 * @param t_index: the time index (indicating the number of predicted map)
 */
void FakeRiskVoxel::addObstacles(const std::vector<Eigen::Vector3d> &centers,
                                 const Eigen::Vector3d              &size,
                                 int                                 t_index) {
  for (auto &pt : centers) {
    float pt_x   = static_cast<float>(pt.x());
    float pt_y   = static_cast<float>(pt.y());
    float pt_z   = static_cast<float>(pt.z());
    float size_x = static_cast<float>(size.x());
    float size_y = static_cast<float>(size.y());
    float size_z = static_cast<float>(size.z());

    for (float z = pt_z - size_z; z <= pt_z + size_z; z += resolution_) {
      for (float y = pt_y - size_y; y <= pt_y + size_y; y += resolution_) {
        for (float x = pt_x - size_x; x <= pt_x + size_x; x += resolution_) {
          Eigen::Vector3f ptf = Eigen::Vector3f(x, y, z);
          if (isInRange(ptf)) {
            int index = getVoxelIndex(ptf);

            risk_maps_[index][t_index] = 1.0;
          }
        }
      }
    }
  }
}

/**
 * @brief add obstacles to the risk map at the given time
 * @param points: given obstacle points in the map frame
 * @param size: the size of the obstacle (axis-aligned bounding box)
 */
void FakeRiskVoxel::addObstacles(const std::vector<Eigen::Vector3d> &centers,
                                 const Eigen::Vector3d              &size,
                                 const ros::Time                    &t) {
  double dt = t.toSec() - last_update_time_.toSec(); /* TODO: use last update time */
  // double dt  = t.toSec() - ros::Time::now().toSec(); /* approximation: use current time */
  int idx = floor(dt / time_resolution_);
  if (idx >= PREDICTION_TIMES) {
    ROS_WARN("[MAP] The time %.2f is too large, the risk map will not be updated", dt);
    return;
  }
  if (idx < 0) {
    ROS_WARN("[MAP] The time %.2f is too small, the risk map will not be updated", dt);
    return;
  }
  addObstacles(centers, size, idx);
}
