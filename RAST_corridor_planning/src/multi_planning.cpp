/**
 * @file multi_planning.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-07-29
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <multi_planning.h>

namespace planner {
// Planner::Planner(ros::NodeHandle& nh, const PlannerConfig& conf) : _nh(nh), _config(conf) {
void Planner::init() {
  /*** ASTAR SETTINGS ***/
  _astar_planner.setTimeParameters(_config.a_star_search_time_step, _config.planning_time_step);
  _astar_planner.setHeightLimit(_config.use_height_limit, _config.height_limit_max,
                                _config.height_limit_min);
  _astar_planner.setIfSampleZDirection(_config.sample_z_acc);
  _astar_planner.setMaximumVelAccAndStep(
      static_cast<float>(_config.max_vel), static_cast<float>(_config.max_vel),
      static_cast<float>(_config.max_acc), static_cast<float>(_config.max_acc / 2.0),
      _config.a_star_acc_sample_step);
  _astar_planner.setRiskThreshold(_config.risk_threshold_motion_primitive,
                                  _config.risk_threshold_single_voxel,
                                  _config.risk_threshold_corridor);

  _ref_direction_angle = 100.f;

  /*** INITIALIZE VISUALIZATION ***/
  std::string frame_id = "world";
  _vis.reset(new Visualizer(_nh, frame_id));

  /*** SUBSCRIBERS ***/
  _future_risk_sub =
      _nh.subscribe("/my_map/future_risk_full_array", 1, &Planner::FutureRiskCallback, this);
  _pose_sub = _nh.subscribe("/mavros/local_position/pose", 1, &Planner::PoseCallback, this);
  _vel_sub = _nh.subscribe("/mavros/local_position/velocity_local", 1, &Planner::VelCallback, this);
  // TODO: no velocity_local message in simulation
  /*** PUBLISHERS ***/
  _traj_pub     = _nh.advertise<traj_utils::PolyTraj>("trajectory", 1);
  _corridor_pub = _nh.advertise<decomp_ros_msgs::DynPolyhedronArray>("corridor", 1);

  ROS_INFO("Wait for 2 seconds");
  ros::Duration(2.0).sleep();

  _traj_timer =
      _nh.createTimer(ros::Duration(_config.planning_time_step), &Planner::TrajTimerCallback, this);
  _traj_idx = 0;
  /*** AUXILIARY VARIABLES ***/
  _prev_vx = 0.0;
  _prev_vy = 0.0;
  _prev_vz = 0.0;

  /*** MAP ***/
  /** @brief map order: z,y,x,t */
  _map_z_limit     = MAP_WIDTH_VOXEL_NUM * MAP_LENGTH_VOXEL_NUM;
  _map_y_limit     = MAP_LENGTH_VOXEL_NUM;
  _map_x_limit     = 1;
  _map_size        = MAP_HEIGHT_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM * MAP_LENGTH_VOXEL_NUM;
  _map_half_length = MAP_LENGTH_VOXEL_NUM * VOXEL_RESOLUTION / 2.f;
  _map_half_width  = MAP_WIDTH_VOXEL_NUM * VOXEL_RESOLUTION / 2.f;
  _map_half_height = MAP_HEIGHT_VOXEL_NUM * VOXEL_RESOLUTION / 2.f;

  /*** BOOLEANS ***/
  _is_rviz_center_locked     = _config.is_rviz_map_center_locked;
  _is_future_risk_updated    = false;
  _is_future_risk_locked     = false;
  _is_safety_mode_enabled    = false;
  _is_odom_received          = false;
  _is_trajectory_initialized = false;
  _is_state_locked           = false;
}

void Planner::FutureRiskCallback(const std_msgs::Float32MultiArrayConstPtr& risk_msg) {
  _is_future_risk_locked = true;
  for (int i = 0; i < VOXEL_NUM; ++i) {
    for (int j = 0; j < RISK_MAP_NUMBER; ++j) {
      _future_risk[i][j] = risk_msg->data[i * risk_msg->layout.dim[0].stride + j];
    }
  }
  _is_future_risk_locked = false;

  _map_center.pose.position.x = risk_msg->data[VOXEL_NUM * RISK_MAP_NUMBER];
  _map_center.pose.position.y = risk_msg->data[VOXEL_NUM * RISK_MAP_NUMBER + 1];
  _map_center.pose.position.z = risk_msg->data[VOXEL_NUM * RISK_MAP_NUMBER + 2];

  _is_future_risk_updated = true;
}

/**
 * @brief get current position and attitude from odometry
 * @param msg
 */
void Planner::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (!_is_state_locked) {
    _is_state_locked = true;
    _odom_pos.x()    = msg->pose.position.x;
    _odom_pos.y()    = msg->pose.position.y;
    _odom_pos.z()    = msg->pose.position.z;

    _odom_att.x()     = msg->pose.orientation.x;
    _odom_att.y()     = msg->pose.orientation.y;
    _odom_att.z()     = msg->pose.orientation.z;
    _odom_att.w()     = msg->pose.orientation.w;
    _is_odom_received = true;
  }
  _is_state_locked = false;

  Eigen::Quaternionf axis;  //= quad * q1 * quad.inverse();
  axis.w()                       = cos(-M_PI / 4.0);
  axis.x()                       = 0.0;
  axis.y()                       = 0.0;
  axis.z()                       = sin(-M_PI / 4.0);
  Eigen::Quaternionf rotated_att = _odom_att * axis;
}

/**
 * @brief Calculate virtual accelerations from velocity. Original accelerations given by px4 is too
 * noisy.
 *
 * @param msg
 */
void Planner::VelCallback(const geometry_msgs::TwistStamped& msg) {
  _odom_vel.x() = msg.twist.linear.x;
  _odom_vel.y() = msg.twist.linear.y;
  _odom_vel.z() = msg.twist.linear.z;

  bool is_vel_initialized = true;

  if (is_vel_initialized) {
    is_vel_initialized = false;
  } else {
    double dt    = ros::Time::now().toSec() - _prev_t;
    _odom_acc(0) = (_odom_vel(0) - _prev_vx) / dt;
    _odom_acc(1) = (_odom_vel(1) - _prev_vy) / dt;
    _odom_acc(2) = (_odom_vel(2) - _prev_vz) / dt;

    if (fabs(_odom_acc(0)) < 0.2) _odom_acc(0) = 0.0;  // dead zone for acc x
    if (fabs(_odom_acc(1)) < 0.2) _odom_acc(1) = 0.0;  // dead zone for acc y
    if (fabs(_odom_acc(2)) < 0.2) _odom_acc(2) = 0.0;  // dead zone for acc z

    for (int i = 0; i < 3; i++) {
      if (_odom_acc(i) < -_config.max_differentiated_current_a) {
        _odom_acc(i) = -_config.max_differentiated_current_a;
      } else if (_odom_acc(i) > _config.max_differentiated_current_a) {
        _odom_acc(i) = _config.max_differentiated_current_a;
      }
    }
    // ROS_INFO("acc=(%f, %f, %f)", _odom_acc(0), _odom_acc(1),
    // _odom_acc(2));
  }
  _prev_t  = ros::Time::now().toSec();
  _prev_vx = _odom_vel(0);
  _prev_vy = _odom_vel(1);
  _prev_vz = _odom_vel(2);
}

/**
 * @brief get the index of the voxel that contains the given position in the map
 *
 * @param p query position
 * @param c map center
 * @return int index of voxel in the map
 */
int Planner::getPointSpatialIndexInMap(const Eigen::Vector3d& p, const Eigen::Vector3d& c) {
  int x = static_cast<int>((p(0) - c(0) + _map_half_length) / VOXEL_RESOLUTION);
  int y = static_cast<int>((p(1) - c(1) + _map_half_width) / VOXEL_RESOLUTION);
  int z = static_cast<int>((p(2) - c(2) + _map_half_height) / VOXEL_RESOLUTION);

  int idx = z * _map_z_limit + y * _map_y_limit + x * _map_x_limit;

  if (idx >= 0 && idx < _map_size) {
    return idx;
  } else {
    return -1;
  }
}

/**
 * @brief Trajectory callback, called in a fixed time rate.
 *
 * @param event
 */
void Planner::TrajTimerCallback(const ros::TimerEvent& event) {
  ROS_WARN("Time interval between two plannings = %lf",
           ros::Time::now().toSec() - _prev_opt_end_time);

  if (!_is_future_risk_updated) return;

  double _traj_planning_start_time = ros::Time::now().toSec();  // TODO: useless?

  /*** Copy future status ***/  // TODO(@chen): WHY?
  // static float future_risk_planning[VOXEL_NUM][RISK_MAP_NUMBER];
  // while (future_risk_locked) {
  //   ros::Duration(0.0001).sleep();
  // }
  // future_risk_locked = true;
  // for (int i = 0; i < VOXEL_NUM; ++i) {
  //   for (int j = 0; j < RISK_MAP_NUMBER; ++j) {
  //     future_risk_planning[i][j] = _future_risk[i][j];
  //   }
  // }
  // future_risk_locked = false;

  /** @brief  the start position of the planned trajectory in map frame*/
  Eigen::Vector3d p_start = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_start = Eigen::Vector3d::Zero();
  Eigen::Vector3d a_start = Eigen::Vector3d::Zero();

  /** @brief map center when trajectory planning start */
  Eigen::Vector3d c_start;
  c_start << _map_center.pose.position.x, _map_center.pose.position.y, _map_center.pose.position.z;

  /***************************************************************************************/
  /***** P1: Check the risk of the planned short trajectory and set a start position ****/
  /*** Initialization ***/

  // TODO revise this after understanding the EGO FSM
  p_start = _odom_pos - c_start;
  v_start = _odom_vel;
  a_start = _odom_acc;

  // /** @brief  0: normal 1: tracking error too large 2: safety mode */
  // geometry_msgs::PointStamped tracking_error_signal;
  // tracking_error_signal.header.stamp = ros::Time::now();
  // tracking_error_signal.point.x      = 0.0;

  // if (_is_safety_mode_enabled) {
  //   p_start = _p_store_for_em - c_start;
  //   v_start = Eigen::Vector3d::Zero();

  //   tracking_error_signal.point.x = 2.0;

  // } else {
  //   /// Calculated risk of planned trajectory
  //   float risk = 0.f;

  //   std::queue<PVAYPoint> current_queue_copy(trajectory_piece);
  //   while (!current_queue_copy.empty()) {
  //     auto p = current_queue_copy.front();

  //     int spatial_index = getPointSpatialIndexInMap(p, c_start);
  //     if (spatial_index >= 0) {
  //       risk += _future_risk[spatial_index][0];
  //     }

  //     current_queue_copy.pop();
  //   }

  //   /// Set planning initial state
  //   if (risk > _config.risk_threshold_motion_primitive) {
  //     p_start = _p_store_for_em - c_start;
  //     // Empty the queue so it will enter safety mode
  //     std::queue<PVAYPoint> empty;
  //     std::swap(trajectory_piece, empty);
  //     ROS_WARN("Current planned trajecotry not safe!");
  //   } else if (!trajectory_piece.empty() && (_p_store_for_em - _odom_pos).norm() > 1.0) {
  //     // Tracking error too large
  //     PVAYPoint temp_p;
  //     temp_p.position     = 0.5 * _p_store_for_em + 0.5 * _odom_pos;
  //     temp_p.velocity     = Eigen::Vector3d::Zero();
  //     temp_p.acceleration = Eigen::Vector3d::Zero();
  //     temp_p.yaw          = 0.0;

  //     // Empty the queue and add the temp point
  //     std::queue<PVAYPoint> empty;
  //     std::swap(trajectory_piece, empty);
  //     trajectory_piece.push(temp_p);

  //     p_start = temp_p.position - c_start;

  //     ROS_WARN("Tracking error too large. Set a new start point.");
  //     tracking_error_signal.point.x = 1.0;
  //   } else {
  //     if (trajectory_piece.size() > _config.trajectory_piece_max_size * 0.8) {
  //       /// Planning is not necessary
  //       ROS_WARN("Planning is not necessary!");
  //       _prev_opt_end_time = ros::Time::now().toSec();
  //       return;
  //     } else if (!trajectory_piece.empty()) {
  //       p_start = trajectory_piece.back().position - c_start;
  //       v_start = trajectory_piece.back().velocity;
  //       a_start = trajectory_piece.back().acceleration;
  //       if (a_start.norm() > _config.max_differentiated_current_a) {
  //         a_start = a_start / a_start.norm() * _config.max_differentiated_current_a;
  //       }
  //     }
  //   }
  // }
  // tracking_error_too_large_state_pub.publish(tracking_error_signal);

  /***************************************************************************************/
  /************** P2: Risk-aware Kino-dynamic A-star planning *****************************/

  double astar_planning_start_time = ros::Time::now().toSec();

  /** truncate the initial velocity */
  if (fabs(v_start.x()) >
      _astar_planner.v_max_xy) {  // TODO(@siyuan): v_start = (v_start > M) ? M : v_start;
    v_start.x() = _astar_planner.v_max_xy * v_start.x() / fabs(v_start.x());
  }
  if (fabs(v_start.y()) > _astar_planner.v_max_xy) {
    v_start.y() = _astar_planner.v_max_xy * v_start.y() / fabs(v_start.y());
  }
  if (fabs(v_start.z()) > _astar_planner.v_max_z) {
    v_start.z() = _astar_planner.v_max_z * v_start.z() / fabs(v_start.z());
  }

  Node* start_node =
      new Node(0, p_start.x(), p_start.y(), p_start.z(), v_start.x(), v_start.y(), v_start.z());
  Node* end_node = new Node(0, _config.goal_x - c_start(0), _config.goal_y - c_start(1),
                            _config.goal_z - c_start(2), 0, 0, 0);

  std::vector<Node*> astar_rst;  // result of A* planning

  // float astar_start_time = trajectory_piece.size() * _config.planning_time_step;
  float astar_start_time = 0;

  _astar_planner.updateMapCenterPosition(c_start(0), c_start(1), c_start(2));
  _astar_planner.search(start_node, end_node, astar_start_time, _config.expand_safety_distance,
                        _ref_direction_angle, &_future_risk[0][0],
                        astar_rst);  // distance = 0.25

  std::vector<TrajPoint> searched_points;
  _astar_planner.getSearchedPoints(searched_points);

  /// Visualize searched points
  //    vector<Eigen::Vector3d> searched_points_to_show;
  //    for(const auto &searched_point : searched_points){
  //        Eigen::Vector3d p;
  //        if(rviz_map_center_locked){
  //            p<<searched_point.x, searched_point.y, searched_point.z;
  //        }else{
  //            p<<searched_point.x+_map_center.pose.position.x,
  //            searched_point.y+_map_center.pose.position.y,
  //            searched_point.z+_map_center.pose.position.z;
  //        }
  //        searched_points_to_show.push_back(p);
  //    }
  //
  //    _vis->visualizeAstarPath(searched_points_to_show, 89, 1.0, 0.1, 0.1, 1.0, 0.05);

  /// To record max and avg search time
  // static double a_star_total_time = 0.0;
  // static double a_star_max_time   = 0.0;
  // static int    a_star_counter    = 0;
  double a_star_time = ros::Time::now().toSec() - astar_planning_start_time;
  // a_star_total_time += a_star_time;
  // a_star_counter += 1;
  // if (a_star_time > a_star_max_time) {
  //   a_star_max_time = a_star_time;
  // }
  ROS_INFO("[PLANNING] A* takes %lf s", a_star_time);
  // ROS_INFO("A* AVG time = %lf", a_star_total_time / (double)a_star_counter);
  // ROS_INFO("A* MAX time = %lf", a_star_max_time);

  if (astar_rst.size() <= 1 || astar_rst.size() >= 10) {
    ROS_WARN("A* planning failed!");
    /// Eliminate the left points in RVIZ
    vector<Eigen::Vector3d> points;
    _vis->visualizeAstarPath(points, 0, 0.8, 0.3, 0.4, 1.0, 0.2, visualization_msgs::Marker::POINTS,
                             true);
    _vis->visualizeAstarPath(points, 1, 0.1, 0.9, 0.2, 1.0, 0.1,
                             visualization_msgs::Marker::LINE_STRIP, true);

    vector<Corridor*> corridors;
    _vis->visualizeCorridors(corridors, _map_center, true);
    return;

    //        /******** TEST code for emergency *********/
    //        std::queue<PVAYPoint> empty;
    //        std::swap( trajectory_piece, empty);
  } else {
    // at least two nodes are generated to build a corridor
    std::vector<Eigen::Vector3d> points;
    for (auto& p : astar_rst) {
      Eigen::Vector3d p_this;
      if (_is_rviz_center_locked) {
        p_this.x() = p->x;
        p_this.y() = p->y;
        p_this.z() = p->z;
      } else {
        p_this.x() = p->x + _map_center.pose.position.x;
        p_this.y() = p->y + _map_center.pose.position.y;
        p_this.z() = p->z + _map_center.pose.position.z;
      }
      points.push_back(p_this);
    }
    _vis->visualizeAstarPath(points, 0, 0.8, 0.3, 0.4, 1.0, 0.2);

    // Set reference direction angle
    _ref_direction_angle = atan2(points[1].y() - points[0].y(), points[1].x() - points[0].x());

    // Restore trajectories and publish
    vector<Eigen::Vector3d> a_star_traj_points_to_show;

    for (int i = 0; i < astar_rst.size() - 1; ++i) {
      auto node1 = astar_rst[i];
      auto node2 = astar_rst[i + 1];

      float ax = (node2->vx - node1->vx) / _astar_planner.time_step_node;
      float ay = (node2->vy - node1->vy) / _astar_planner.time_step_node;
      float az = (node2->vz - node1->vz) / _astar_planner.time_step_node;
      auto  point_num_one_piece =
          (int)(_astar_planner.time_step_node / _astar_planner.time_step_trajectory);

      for (int j = 1; j < point_num_one_piece;
           ++j) {  // Skip the first point. Which is the same as the last point on the last piece.
        Eigen::Vector3d p;
        float           t = (float)j * _astar_planner.time_step_trajectory;
        p.x()             = node1->x + node1->vx * t + 0.5 * ax * t * t;
        p.y()             = node1->y + node1->vy * t + 0.5 * ay * t * t;
        p.z()             = node1->z + node1->vz * t + 0.5 * az * t * t;

        if (!_is_rviz_center_locked) {
          p.x() += _map_center.pose.position.x;
          p.y() += _map_center.pose.position.y;
          p.z() += _map_center.pose.position.z;
        }
        a_star_traj_points_to_show.push_back(p);
      }
    }
    _vis->visualizeAstarPath(a_star_traj_points_to_show, 1, 0.1, 0.9, 0.2, 1.0, 0.1,
                             visualization_msgs::Marker::LINE_STRIP);

    /***** P3: Risk-constrained corridor ****/
    double            corridor_start_time = ros::Time::now().toSec();
    vector<Corridor*> corridors;

    _astar_planner.findCorridors(corridors, 2);

    /// To record max and avg corridors calculation time
    // static double corridor_total_time = 0.0;
    // static double corridor_max_time   = 0.0;
    // static int    corridor_counter    = 0;
    double corridor_time = ros::Time::now().toSec() - corridor_start_time;
    // corridor_total_time += corridor_time;
    // corridor_counter += 1;
    // if (corridor_time > corridor_max_time) {
    //   corridor_max_time = corridor_time;
    // }
    ROS_INFO("[PLANNING]: corridor generation takes %lf s", corridor_time);
    // ROS_INFO("corridors AVG time = %lf", corridor_total_time / (double)corridor_counter);
    // ROS_INFO("corridors MAX time = %lf", corridor_max_time);

    /// Publish corridors to optimization planner
    decomp_ros_msgs::DynPolyhedronArray corridor_msg;
    corridor_msg.header.stamp = ros::Time::now();
    corridor_msg.start_pos.x  = astar_rst[0]->x;
    corridor_msg.start_pos.y  = astar_rst[0]->y;
    corridor_msg.start_pos.z  = astar_rst[0]->z;
    corridor_msg.start_vel.x  = astar_rst[0]->vx;
    corridor_msg.start_vel.y  = astar_rst[0]->vy;
    corridor_msg.start_vel.z  = astar_rst[0]->vz;
    corridor_msg.start_acc.x  = a_start.x();
    corridor_msg.start_acc.y  = a_start.y();
    corridor_msg.start_acc.z  = a_start.z();

    corridor_msg.end_pos.x = astar_rst[astar_rst.size() - 1]->x;
    corridor_msg.end_pos.y = astar_rst[astar_rst.size() - 1]->y;
    corridor_msg.end_pos.z = astar_rst[astar_rst.size() - 1]->z;
    corridor_msg.end_vel.x = astar_rst[astar_rst.size() - 1]->vx;
    corridor_msg.end_vel.y = astar_rst[astar_rst.size() - 1]->vy;
    corridor_msg.end_vel.z = astar_rst[astar_rst.size() - 1]->vz;

    corridor_msg.end_acc.x = 0.f;
    corridor_msg.end_acc.y = 0.f;
    corridor_msg.end_acc.z = 0.f;

    for (auto& corridor : corridors) {
      decomp_ros_msgs::DynPolyhedron corridor_this;
      corridor_this.duration = _config.a_star_search_time_step;
      for (const auto& surface : corridor->envelope.surfaces) {
        geometry_msgs::Point point;
        geometry_msgs::Point normal;
        point.x  = surface.point.x;
        point.y  = surface.point.y;
        point.z  = surface.point.z;
        normal.x = surface.normal.x;
        normal.y = surface.normal.y;
        normal.z = surface.normal.z;
        corridor_this.points.push_back(point);
        corridor_this.normals.push_back(normal);
      }
      corridor_msg.dyn_polyhedrons.push_back(corridor_this);
    }
    _corridor_pub.publish(corridor_msg);
    _vis->visualizeCorridors(corridors, _map_center, _is_rviz_center_locked);

    /***** P4: Trajectory Optimization *****/
    double optimization_start_time = ros::Time::now().toSec();

    /** extract init final states from the corridors **/
    Eigen::Matrix3d init_state, final_state;
    init_state.col(0) << astar_rst[0]->x, astar_rst[0]->y, astar_rst[0]->z;
    init_state.col(1) << astar_rst[0]->vx, astar_rst[0]->vy, astar_rst[0]->vz;
    init_state.col(2) << a_start.x(), a_start.y(), a_start.z();
    int n = astar_rst.size() - 1;
    final_state.col(0) << astar_rst[n]->x, astar_rst[n]->y, astar_rst[n]->z;
    final_state.col(1) << astar_rst[n]->vx, astar_rst[n]->vy, astar_rst[n]->vz;
    final_state.col(2) << 0.f, 0.f, 0.f;

    /** convert corridor (Corridor) to polyhedra (Eigen::Matrix) **/
    std::vector<Eigen::Matrix<double, 6, -1>> polyhedra;
    std::vector<double>                       time_alloc;
    polyhedra.reserve(corridors.size());
    time_alloc.reserve(corridors.size());
    for (auto& corridor : corridors) {
      Eigen::MatrixXd polygon;
      int             s = corridor->envelope.surfaces.size();
      polygon.resize(6, s);
      for (unsigned int i = 0; i < s; i++) {
        polygon.col(i).tail<3>() << corridor->envelope.surfaces[i].point.x,
            corridor->envelope.surfaces[i].point.y, corridor->envelope.surfaces[i].point.z;
        polygon.col(i).head<3>() << corridor->envelope.surfaces[i].normal.x,
            corridor->envelope.surfaces[i].normal.y, corridor->envelope.surfaces[i].normal.z;
      }
      polyhedra.push_back(polygon);
      time_alloc.push_back((double)_config.a_star_search_time_step);
    }

    std::cout << "corridor size: " << corridors.size() << std::endl;
    std::cout << "polyhedra size: " << polyhedra.size() << std::endl;

    /** apply optimization **/
    // bool is_trajectory_optimized =
    //     OptimizationInCorridors(polyhedra, time_alloc, init_state, final_state);
    bool is_solved = false;
    _traj_duration = 0; /** Total allocated time among input corridors */
    for (auto it = time_alloc.begin(); it != time_alloc.end(); ++it) {
      _traj_duration += (*it);
    }
    std::cout << "Piece num:" << time_alloc.size() << std::endl;
    std::cout << "Total time: " << _traj_duration << std::endl;

    /* initial optimize */
    _traj_optimizer.reset(init_state, final_state, time_alloc, polyhedra);
    try {
      is_solved = _traj_optimizer.optimize(_config.factors, _config.delta_corridor);
    } catch (int e) {
      ROS_ERROR("Optimizer crashed!");
      // return false;
    }

    if (is_solved) {
      _traj_optimizer.getTrajectory(&_traj);
    } else {
      ROS_ERROR("No solution found for these corridors!");
      // return false;
    }

    /* re-optimize */
    int I = 10;  // max iterations
    int i = 0;
    while (!_traj_optimizer.isCorridorSatisfied(_traj, _config.max_vel_optimization,
                                                _config.max_acc_optimization,
                                                _config.delta_corridor) &&
           i++ < I) {
      try {
        is_solved = _traj_optimizer.reOptimize();
      } catch (int e) {
        ROS_ERROR("Optimizer crashed!");
        // return false;
      }

      if (is_solved) {
        _traj_optimizer.getTrajectory(&_traj);
      } else {
        ROS_ERROR("No solution found for these corridors!");
        // return false;
      }
    }
    ROS_INFO("Re-optimization %d", i);
    bool is_trajectory_optimized = is_solved;
    _prev_opt_end_time           = ros::Time::now().toSec();
    ROS_INFO("[PLANNING] trajectory optimization takes %lf s",
             ros::Time::now().toSec() - optimization_start_time);
    if (!is_trajectory_optimized) {
      ROS_WARN("Optimization failed!!");
    }
  }  // end of if a_star valid
  _traj_start_time = ros::Time::now();
  _traj_end_time   = ros::Time::now() + ros::Duration(_traj_duration);
  publishTrajectory();
}

/**
 * @brief optimization function
 *
 * @param msg
 * @param c_start map center in world frame
 * @return true
 * @return false
 */
bool Planner::OptimizationInCorridors(const std::vector<Eigen::Matrix<double, 6, -1>>& c,
                                      const std::vector<double>&                       t,
                                      const Eigen::Matrix3d&                           init,
                                      const Eigen::Matrix3d&                           final) {
  bool   is_solved = false;
  double T         = 0; /** Total allocated time among input corridors */
  for (auto it = t.begin(); it != t.end(); ++it) {
    T += (*it);
  }
  std::cout << "Piece num:" << c.size() << std::endl;
  std::cout << "Total time: " << T << std::endl;

  /* initial optimize */
  _traj_optimizer.reset(init, final, t, c);
  try {
    is_solved = _traj_optimizer.optimize(_config.factors, _config.delta_corridor);
  } catch (int e) {
    ROS_ERROR("Optimizer crashed!");
    return false;
  }

  if (is_solved) {
    _traj_optimizer.getTrajectory(&_traj);
  } else {
    ROS_ERROR("No solution found for these corridors!");
    return false;
  }

  /* re-optimize */
  int I = 5;  // max iterations
  int i = 0;
  while (!_traj_optimizer.isCorridorSatisfied(_traj, _config.max_vel_optimization,
                                              _config.max_acc_optimization,
                                              _config.delta_corridor) &&
         i++ < I) {
    try {
      is_solved = _traj_optimizer.reOptimize();
    } catch (int e) {
      ROS_ERROR("Optimizer crashed!");
      return false;
    }

    if (is_solved) {
      _traj_optimizer.getTrajectory(&_traj);
    } else {
      ROS_ERROR("No solution found for these corridors!");
      return false;
    }
  }
  ROS_INFO("Re-optimization %d", i);
  _traj_idx++;

  /* publish trajectory */
  publishTrajectory();
}

/**
 * @brief Publish the trajectory
 *
 */
void Planner::publishTrajectory() {
  traj_utils::PolyTraj poly_msg;
  int                  piece_num = _traj.getPieceNum();

  poly_msg.drone_id   = 0;
  poly_msg.traj_id    = _traj_idx;
  poly_msg.start_time = _traj_start_time;  // TODO:
  poly_msg.order      = 7;
  poly_msg.duration.resize(piece_num);
  poly_msg.coef_x.resize(8 * piece_num);
  poly_msg.coef_y.resize(8 * piece_num);
  poly_msg.coef_z.resize(8 * piece_num);

  for (int i = 0; i < piece_num; i++) {
    poly_msg.duration[i] = _traj[i].getDuration();

    Eigen::Matrix<double, 3, 8> coef = _traj[i].getCoefficient();
    int  idx  = i * 8;
    for (int j = 0; j < 8; j++) {
      poly_msg.coef_x[idx + j] = coef.row(0)[j];
      poly_msg.coef_y[idx + j] = coef.row(1)[j];
      poly_msg.coef_z[idx + j] = coef.row(2)[j];
    }
  }

  _traj_pub.publish(poly_msg);
}

}  // namespace planner
