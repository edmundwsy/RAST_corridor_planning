/**
 * @file baseline.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-24
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <plan_manager/baseline.h>
#include <ros/ros.h>

void BaselinePlanner::init() {
  /*** INITIALIZE MAP ***/
  map_.reset(new RiskVoxel());
  map_->init(nh_);

  /*** INITIALIZE A STAR ***/
  a_star_.reset(new RiskHybridAstar());
  a_star_->setParam(nh_);
  a_star_->setEnvironment(map_);
  a_star_->init(Eigen::Vector3d::Zero(), Eigen::Vector3d(10, 10, 4));  // TODO: odom_pos_?

  /*** INITIALIZE BEZIER OPT ***/
  traj_optimizer_.reset(new traj_opt::BezierOpt());
  ROS_INFO("Trajectory optimizer initialized.");

  /*** INITIALIZE MADER DECONFLICTION ***/
  collision_avoider_.reset(new ParticleATC(nh_));
  collision_avoider_->init();
  map_->setCoordinator(collision_avoider_);

  /*** INITIALIZE VISUALIZATION ***/
  std::string ns = "world";
  visualizer_.reset(new visualizer::Visualizer(nh_, ns));

  /*** INITIALIZE SUBSCRIBER ***/
  obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("vis_obstacle", 100);

  ROS_INFO("Baseline planner initialized");
}

/**
 * @brief show A star results
 *
 */
void BaselinePlanner::showAstarPath() {
  std::vector<Eigen::Vector3d> path = a_star_->getPath(0.1);
  visualizer_->visualizeAstarPath(path);
  visualizer_->visualizeAstarPathXYT(path, 0.1);
}

/**
 * @brief display obstacles
 *
 * @param points buffer of points
 */
void BaselinePlanner::showObstaclePoints(const std::vector<Eigen::Vector3d> &points) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < static_cast<int>(points.size()); i++) {
    pcl::PointXYZ p;
    p.x = points[i](0);
    p.y = points[i](1);
    p.z = points[i](2);
    cloud.push_back(p);
  }
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp    = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  obstacle_pub_.publish(cloud_msg);
}

/**
 * @brief get empty trajectory
 *
 * If all the Bezier control points are the same, the trajectory is a point.
 * @param pos positions of Bezier control points
 */
void BaselinePlanner::setEmptyTrajectory(const Eigen::Vector3d &pos) {
  traj_ = Bernstein::Bezier(cfg_.corridor_tau);

  Eigen::Matrix<double, 5, 3> cpts;
  cpts.row(0) = pos;
  cpts.row(1) = pos;
  cpts.row(2) = pos;
  cpts.row(3) = pos;
  cpts.row(4) = pos;

  traj_.setControlPoints(cpts);
}

/**
 * @brief generate a axis-aligned cuboid as the initial corridor
 *
 * @param left_higher_corner left, front, higher corner, (x, y, z) maximum
 * @param right_lower_corner right, back, lower corner, (x, y, z) minimum
 * @return H-representation of the cuboid
 */
Eigen::Matrix<double, 6, 4> BaselinePlanner::getInitCorridor(
    const Eigen::Vector3d &left_higher_corner, const Eigen::Vector3d &right_lower_corner) {
  Eigen::Matrix<double, 6, 4> corridor = Eigen::Matrix<double, 6, 4>::Zero();

  corridor(0, 0) = 1.0;
  corridor(1, 1) = 1.0;
  corridor(2, 2) = 1.0;
  corridor(3, 0) = -1.0;
  corridor(4, 1) = -1.0;
  corridor(5, 2) = -1.0;

  corridor.block<3, 1>(0, 3) = -left_higher_corner;
  corridor.block<3, 1>(3, 3) = right_lower_corner;
  return corridor;
}

// /**
//  * @brief add agents trajectory to map
//  * get agents trajectory from collision avoider
//  * add agents trajectory to map
//  */
// void BaselinePlanner::addAgentsTrajectoryToMap() {
//   int             n = collision_avoider_->getNumAgents();
//   Eigen::Vector3d robot_size;
//   collision_avoider_->getAgentsSize(robot_size);
//   std::vector<Eigen::Vector3d> obs_points;
//   for (int i = 0; i < n - 1; i++) {
//     obs_points.clear();
//     collision_avoider_->getAgentsTrajectory(obs_points, i, 0.2);
//     if (obs_points.size() > 0) {
//       for (int j = 0; j < obs_points.size(); j++) {
//         std::vector<Eigen::Vector3d> pts;
//         pts.push_back(obs_points[j] - odom_pos_);
//         map_->addObstacles(pts, robot_size, ros::Time::now() + ros::Duration(0.2 * j));
//       }
//     }
//   }
// }

bool BaselinePlanner::replan(double                 t,
                             const Eigen::Vector3d &start_pos,
                             const Eigen::Vector3d &start_vel,
                             const Eigen::Vector3d &start_acc,
                             const Eigen::Vector3d &goal_pos) {
  ROS_INFO("Replanning ... start position (%f, %f, %f)", start_pos(0), start_pos(1), start_pos(2));
  traj_start_time_ = t;

  // TODO: check time system
  ros::Time t0, t1;
  t0 = ros::Time::now();

  /*----- Path Searching on DSP Dynamic -----*/
  std::cout << "/*----- Path Searching on DSP Static -----*/" << std::endl;
  a_star_->reset();
  t1                 = ros::Time::now();
  t0                 = map_->getMapTime();
  double t_after_map = (t1 - t0).toSec();
  ROS_INFO("[Astar] start position on map: %f | %i", t_after_map,
           static_cast<int>((t1 - t0).toSec() / cfg_.corridor_tau));
  ASTAR_RET rst = a_star_->search(start_pos, start_vel, start_acc, goal_pos,
                                  Eigen::Vector3d(0, 0, 0), true, true, t_after_map);
  if (rst == 0) {
    t1                 = ros::Time::now();
    t0                 = map_->getMapTime();
    double t_after_map = (t1 - t0).toSec();
    a_star_->reset();
    rst = a_star_->search(start_pos, start_vel, start_acc, goal_pos, Eigen::Vector3d(0, 0, 0),
                          false, true, t_after_map);
  }

  auto t2 = ros::Time::now();
  ROS_INFO("[Astar] cost: %f ms", (t2 - t1).toSec() * 1000);

  /* if no path found, set empty trajectory */
  if (rst == NO_PATH) {
    ROS_ERROR("[Astar] No path found!");
    return false;
  }
  showAstarPath();  // visualization

  /*----- Safety Corridor Generation -----*/
  std::cout << "/*----- Safety Corridor Generation -----*/" << std::endl;
  std::vector<Eigen::Matrix<double, 6, 1>> route_vel = a_star_->getPathWithVel(cfg_.corridor_tau);
  // std::vector<Eigen::Vector3d>             route     = a_star_->getPath(cfg_.corridor_tau);
  std::vector<Eigen::Vector3d>  wpts;
  std::vector<Eigen::MatrixX4d> hPolys;

  wpts.resize(route_vel.size()); /* copy route_vel to route */

  std::cout << "start_position = \t" << start_pos.transpose() << std::endl;
  for (int i = 0; i < wpts.size(); i++) {
    wpts[i] = route_vel[i].head(3); /* copy position to route */
    std::cout << "local_wpts[" << i << "] = " << wpts[i].transpose() << std::endl;
    if (wpts[i].z() < 0) wpts[i].z() = 0.1;
  }

  std::vector<Eigen::Vector3d> pc;
  pc.reserve(2000);

  Eigen::Vector3d lower_corner  = Eigen::Vector3d(-4, -4, -1) + start_pos;
  Eigen::Vector3d higher_corner = Eigen::Vector3d(4, 4, 1) + start_pos;
  if (lower_corner.z() < 0) lower_corner.z() = 0;
  if (higher_corner.z() > 4) higher_corner.z() = 4;
  Eigen::Matrix<double, 6, 4> init_corridor = getInitCorridor(higher_corner, lower_corner);

  t1 = ros::Time::now();
  for (int i = 0; i < wpts.size() - 1; i++) {
    /* Get a local bounding box */
    Eigen::Vector3d llc, lhc; /* local lower corner and higher corner */
    lhc(0) = std::min(std::max(wpts[i](0), wpts[i + 1](0)) + cfg_.init_range, higher_corner(0));
    lhc(1) = std::min(std::max(wpts[i](1), wpts[i + 1](1)) + cfg_.init_range, higher_corner(1));
    lhc(2) = std::min(std::max(wpts[i](2), wpts[i + 1](2)) + cfg_.init_range, higher_corner(2));
    llc(0) = std::max(std::min(wpts[i](0), wpts[i + 1](0)) - cfg_.init_range, lower_corner(0));
    llc(1) = std::max(std::min(wpts[i](1), wpts[i + 1](1)) - cfg_.init_range, lower_corner(1));
    llc(2) = std::max(std::min(wpts[i](2), wpts[i + 1](2)) - cfg_.init_range, lower_corner(2));
    Eigen::Matrix<double, 6, 4> bd = init_corridor;
    bd.block<3, 1>(0, 3)           = -lhc;
    bd.block<3, 1>(3, 3)           = llc;

    pc.clear();
    std::cout << "[dbg] t1_glb = " << traj_start_time_ + i * cfg_.corridor_tau << std::endl;
    double t1_glb = traj_start_time_ + i * cfg_.corridor_tau;
    double t2_glb = traj_start_time_ + (i + 1) * cfg_.corridor_tau;
    map_->getObstaclePoints(pc, t1_glb, t2_glb, llc, lhc);
    std::cout << "pc.size() = " << pc.size() << std::endl;

    Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> m_pc(pc[0].data(), 3,
                                                                         pc.size());

    ros::Time        t3 = ros::Time::now();
    Eigen::MatrixX4d hPoly;
    Eigen::Vector3d  r = Eigen::Vector3d::Ones();
    firi::firi(bd, m_pc, wpts[i], wpts[i + 1], hPoly, r, 2);
    ros::Time t4 = ros::Time::now();
    if (r.x() * r.y() * r.z() < cfg_.min_volumn) {
      std::cout << "Narrow corridor, reject! Ellipsoid radius " << r.transpose()
                << " volumn: " << r.x() * r.y() * r.z() << std::endl;
      this->showObstaclePoints(pc);  // visualization
      break;
    }

    ROS_INFO("[FIRI] %ith corridor takes %f ms", i, (t4 - t3).toSec() * 1000);
    hPolys.push_back(hPoly);
  }
  this->showObstaclePoints(pc);  // visualization

  t2 = ros::Time::now();
  ROS_INFO("[CrdGen] Gen %i corridors cost: %f ms", hPolys.size(), (t2 - t1).toSec() * 1000);
  visualizer_->visualizePolytope(hPolys);

  if (hPolys.size() == 0) {
    ROS_ERROR("Cannot find safety corridors!");
    return false;
  }

  /*----- Trajectory Optimization -----*/
  std::cout << "/*----- Trajectory Optimization -----*/" << std::endl;
  // std::cout << "route size: " << route.size() << std::endl;

  /* Goal position and time allocation */
  Eigen::Vector3d local_goal_pos = route_vel.back().head(3);
  Eigen::Vector3d local_goal_vel = route_vel.back().tail(3);

  std::vector<double> time_alloc;
  time_alloc.resize(hPolys.size(), cfg_.corridor_tau);
  std::cout << "time_alloc size: " << time_alloc.size() << std::endl;
  traj_optimizer_.reset(new traj_opt::BezierOpt());
  Eigen::Matrix3d init_state, final_state;
  init_state.row(0)  = start_pos;
  init_state.row(1)  = start_vel;
  init_state.row(2)  = start_acc;
  final_state.row(0) = local_goal_pos;
  final_state.row(1) = local_goal_vel;
  final_state.row(2) = Eigen::Vector3d(0, 0, 0);

  // std::cout << "init_state: " << init_state << std::endl;
  // std::cout << "final_state: " << final_state << std::endl;
  visualizer_->visualizeStartGoal(start_pos);       // visualization
  visualizer_->visualizeStartGoal(local_goal_pos);  // visualization

  t1 = ros::Time::now();
  traj_optimizer_->setup(init_state, final_state, time_alloc, hPolys, cfg_.opt_max_vel,
                         cfg_.opt_max_acc);
  if (!traj_optimizer_->optimize()) {
    t2 = ros::Time::now();
    ROS_INFO("[TrajOpt] cost: %f ms", (t2 - t1).toSec() * 1000);
    ROS_ERROR("Trajectory optimization failed!");
    return false;
  }
  t2 = ros::Time::now();
  ROS_INFO("[TrajOpt] cost: %f ms", (t2 - t1).toSec() * 1000);

  traj_optimizer_->getOptBezier(traj_);

  /*----- Trajectory Deconfliction -----*/
  // t1 = ros::Time::now();
  // if (!collision_avoider_->isSafeAfterOpt(traj_)) {
  //   ROS_ERROR("Trajectory collides after optimization!");
  //   t2 = ros::Time::now();
  //   ROS_INFO("[MADER] cost: %f ms", (t2 - t1).toSec() * 1000);
  //   return false;
  // }
  // if (!collision_avoider_->isSafeAfterChk()) {
  //   ROS_ERROR("Trajectory published while checking!");
  //   t2 = ros::Time::now();
  //   ROS_INFO("[MADER] cost: %f ms", (t2 - t1).toSec() * 1000);
  //   return false;
  // }
  // t2 = ros::Time::now();
  // ROS_INFO("[MADER] cost: %f ms", (t2 - t1).toSec() * 1000);
  prev_traj_start_time_ = traj_start_time_;
  return true;
}
