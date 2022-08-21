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
  _vis.reset(new visualizer::Visualizer(_nh, frame_id));

  /*** SUBSCRIBERS ***/
  _future_risk_sub = _nh.subscribe("future_risk_full_array", 1, &Planner::FutureRiskCallback, this);
  _pose_sub        = _nh.subscribe("/mavros/local_position/pose", 10, &Planner::PoseCallback, this);
  _vel_sub =
      _nh.subscribe("/mavros/local_position/velocity_local", 10, &Planner::VelCallback, this);
  _trigger_sub = _nh.subscribe("/traj_start_trigger", 1, &Planner::TriggerCallback, this);

  /*** PUBLISHERS ***/
  _traj_pub     = _nh.advertise<traj_utils::PolyTraj>("trajectory", 1);
  _corridor_pub = _nh.advertise<decomp_ros_msgs::DynPolyhedronArray>("corridor", 1);

  ROS_INFO("Wait for 2 seconds");
  ros::Duration(2.0).sleep();

  _traj_timer =
      _nh.createTimer(ros::Duration(_config.planning_time_step), &Planner::FSMCallback, this);
  _traj_idx = 0;

  /*** AUXILIARY VARIABLES ***/
  _prev_pt = ros::Time::now().toSec();
  _prev_px = 0.0;
  _prev_py = 0.0;
  _prev_pz = 0.0;
  _prev_vt = ros::Time::now().toSec();
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
  _is_local_frame         = _config.is_rviz_map_center_locked;
  _is_future_risk_updated = false;
  _is_future_risk_locked  = false;
  _is_safety_mode_enabled = false;
  _is_odom_received       = false;
  _is_exec_triggered      = false;
  _is_state_locked        = false;

  /*** STATE ***/
  _status          = FSM_STATUS::INIT;
  _traj_start_time = ros::Time::now();
  ROS_INFO("[PLANNING] Initialization complete");
}

/**
 * @brief finite state machine for planning
 *  States:
 * - INIT: waiting for input information
 * - WAIT_TARGET: waiting for target information
 * - NEW_PLAN: planning a new trajectory
 * - REPLAN: replanning based on current trajectory
 * - EXEC_TRAJ: executing the trajectory
 * - EMERGENCY_STOP: emergency stop
 * - EXIT: exit the planner
 * @param event
 */
void Planner::FSMCallback(const ros::TimerEvent& event) {
  double risk = 0;
  switch (_status) {
    /* initialize */
    case INIT:
      FSMChangeState(FSM_STATUS::WAIT_TARGET);
      break;

    /* wait for callback */
    case WAIT_TARGET:
      if (_is_future_risk_updated && _is_odom_received) {
        FSMChangeState(FSM_STATUS::NEW_PLAN);
      } else {
        ROS_INFO_ONCE("[PLANNER] Waiting for odom[%d] and future risk[%d] ", _is_odom_received,
                      _is_future_risk_updated);
      }
      break;

    /* plan a new trajectory from current position */
    case NEW_PLAN:
      if (!_is_future_risk_updated || !_is_odom_received) {
        FSMChangeState(FSM_STATUS::WAIT_TARGET);
      } else {
        bool is_success = false;
        if (checkTimeLapse(1.0)) {
          is_success = localReplan(PLAN_TYPE::NEW);
        }
        // bool is_safe    = checkTrajectoryRisk(_traj);
        bool is_safe = true;         // TODO: checkTrajectoryRisk(_traj);
        if (is_success && is_safe) { /* publish trajectory */
          publishTrajectory(_traj);
          ROS_WARN("%f", _traj_start_time.toSec());
        }
        if (_is_exec_triggered) {
          FSMChangeState(FSM_STATUS::EXEC_TRAJ);
        }
      }
      break;

    /* execute the trajectory, replan when current traj is about to finish */
    case EXEC_TRAJ:
      if (!_is_future_risk_updated || !_is_odom_received) {
        FSMChangeState(FSM_STATUS::WAIT_TARGET);
      } else {
        // bool is_safe            = checkTrajectoryRisk(_traj);
        bool is_replan_required = executeTrajectory();
        if (is_replan_required) { /* replan */
          FSMChangeState(FSM_STATUS::REPLAN);
        }
      }
      break;

    /* replan based on current trajectory */
    case REPLAN:
      if (!_is_future_risk_updated || !_is_odom_received) {
        FSMChangeState(FSM_STATUS::WAIT_TARGET);
      } else {
        bool is_success = localReplan(PLAN_TYPE::CONTINUE);
        bool is_safe    = true;  // TODO: checkTrajectoryRisk(_traj);
        // bool is_safe    = checkTrajectoryRisk(_traj);
        if (is_success && is_safe) { /* publish trajectory */
          publishTrajectory(_traj);
          FSMChangeState(FSM_STATUS::EXEC_TRAJ);
        } else {
          ROS_WARN("Replanning failed");
        }
      }
      break;

    /* emergency stop */
    case EMERGENCY_STOP:
      if (!_is_safety_mode_enabled) {
        FSMChangeState(FSM_STATUS::NEW_PLAN);
      }
      break;

    case EXIT:
      break;

    default:
      ROS_ERROR("[FSM] Invalid FSM state");
      break;
  }
}

/**
 * @brief change the state of the finite state machine
 * @param state
 */
void Planner::FSMChangeState(FSM_STATUS new_state) {
  FSMPrintState(new_state);
  _status = new_state;
}

/**
 * @brief print the current state of the finite state machine via termcolor
 * This function is used for debugging purposes
 */
void Planner::FSMPrintState(FSM_STATUS new_state) {
  static string state_str[7] = {"INIT",      "WAIT_TARGET",    "NEW_PLAN", "REPLAN",
                                "EXEC_TRAJ", "EMERGENCY_STOP", "EXIT"};
  std::cout << termcolor::dark << termcolor::on_bright_green << "[FSM] status "
            << termcolor::bright_cyan << termcolor::on_white << state_str[static_cast<int>(_status)]
            << " >> " << state_str[static_cast<int>(new_state)] << termcolor::reset << std::endl;
}

void Planner::TriggerCallback(const geometry_msgs::PoseStampedPtr& msg) {
  if (_is_exec_triggered) {
    return;
  }
  ROS_WARN("[PLANNER] trigger received");
  _traj_start_time   = ros::Time::now() + ros::Duration(0.1);
  _traj_end_time     = _traj_start_time + ros::Duration(_traj_duration);
  _is_exec_triggered = true;
}

/**
 * @brief callback for future risk
 * @param msg
 */
void Planner::FutureRiskCallback(const std_msgs::Float32MultiArrayConstPtr& risk_msg) {
  _is_future_risk_locked = true;
  for (int i = 0; i < VOXEL_NUM; ++i) {
    for (int j = 0; j < RISK_MAP_NUMBER; ++j) {
      _future_risk[i][j] = risk_msg->data[i * risk_msg->layout.dim[0].stride + j];
    }
  }
  _is_future_risk_locked = false;
  if (!_is_local_frame) {
    _map_center.x() = risk_msg->data[VOXEL_NUM * RISK_MAP_NUMBER];
    _map_center.y() = risk_msg->data[VOXEL_NUM * RISK_MAP_NUMBER + 1];
    _map_center.z() = risk_msg->data[VOXEL_NUM * RISK_MAP_NUMBER + 2];
  } else {
    _map_center.x() = 0.f;
    _map_center.y() = 0.f;
    _map_center.z() = 0.f;
  }

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

  // Eigen::Quaternionf axis;  //= quad * q1 * quad.inverse();
  // axis.w()                       = cos(-M_PI / 4.0);
  // axis.x()                       = 0.0;
  // axis.y()                       = 0.0;
  // axis.z()                       = sin(-M_PI / 4.0);
  // Eigen::Quaternionf rotated_att = _odom_att * axis;
  if (!_is_velocity_received) {
    double dt     = msg->header.stamp.toSec() - _prev_pt;
    _odom_vel.x() = (_odom_pos.x() - _prev_px) / dt;
    _odom_vel.y() = (_odom_pos.y() - _prev_py) / dt;
    _odom_vel.z() = (_odom_pos.z() - _prev_pz) / dt;

    _prev_pt = msg->header.stamp.toSec();
    _prev_px = _odom_pos.x();
    _prev_py = _odom_pos.y();
    _prev_pz = _odom_pos.z();
  }
}

/**
 * @brief Calculate virtual accelerations from velocity. Original accelerations given by px4 is
 * too noisy.
 *
 * @param msg
 */
void Planner::VelCallback(const geometry_msgs::TwistStamped& msg) {
  _is_velocity_received = true;

  _odom_vel.x() = msg.twist.linear.x;
  _odom_vel.y() = msg.twist.linear.y;
  _odom_vel.z() = msg.twist.linear.z;

  double dt    = msg.header.stamp.toSec() - _prev_vt;
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
  // ROS_INFO("acc=(%f, %f, %f)", _odom_acc(0), _odom_acc(1), _odom_acc(2));

  _prev_vt = msg.header.stamp.toSec();
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
 * @brief trajectory optimization function
 *
 * @param c
 * @param t
 * @param init
 * @return true
 * @return false
 */
bool Planner::OptimizationInCorridors(const std::vector<Eigen::Matrix<double, 6, -1>>& c,
                                      const std::vector<double>&                       t,
                                      const Eigen::Matrix3d&                           init,
                                      const Eigen::Matrix3d&                           final) {
  bool is_solved = false;

  /* initial optimize */
  _traj_optimizer.reset(init, final, t, c);
  double delta = 0.0;
  is_solved    = _traj_optimizer.optimize(delta);

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
  return true;
}

/**
 * @brief plan a trajectory
 * @param type
 * NEW,      plan a new trajectory from current position
 * CONTINUE, continue the current trajectory from the final position
 * EMERGENCY emergency replan from current position
 * @return true if a trajectory is planned successfully
 * @return false
 */
bool Planner::localReplan(PLAN_TYPE type) {
  bool is_success = false;

  /** @brief  the start position of the planned trajectory in map frame*/
  Eigen::Vector3d p_start, v_start, a_start;

  /** @brief map center when trajectory planning start */
  Eigen::Vector3d c_start = _map_center;

  /**********************************************************/
  /***** STEP1: Clarify Initial States *****/
  /**********************************************************/
  if (type == PLAN_TYPE::NEW) {
    p_start          = _odom_pos - c_start;
    v_start          = Eigen::Vector3d::Zero();
    a_start          = Eigen::Vector3d::Zero();
    _traj_start_time = ros::Time::now();
  } else if (type == PLAN_TYPE::CONTINUE) {
    // continue from the end of the previous trajectory
    double T         = _traj_duration;
    p_start          = _traj.getPos(T) - c_start;
    v_start          = _traj.getVel(T);
    a_start          = _traj.getAcc(T);
    _traj_start_time = _traj_end_time;
    // /* visualize previous trajectory */
    // _vis->visualizeTrajectory(Eigen::Vector3d(0, 0, 0), _traj, _traj.getMaxVelRate());
  } else if (type == PLAN_TYPE::EMERGENCY) {
    p_start          = _odom_pos - c_start;
    v_start          = _odom_vel;
    a_start          = _odom_acc;
    _traj_start_time = ros::Time::now();
  } else {
    ROS_ERROR("Invalid plan type!");
    return false;
  }

  /**********************************************************/
  /***** STEP2: Risk-aware Kino-dynamic A-star planning *****/
  /**********************************************************/
  // NOTE: A-star planning is in the map frame.

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
  double a_star_time = ros::Time::now().toSec() - astar_planning_start_time;
  // ROS_INFO("[PLANNING] A* takes %lf s", a_star_time);

  if (astar_rst.size() <= 1 || astar_rst.size() >= 10) {
    // ROS_WARN("A* planning failed!");
    return is_success;
  } else {
    // at least two nodes are generated to build a corridor
    std::vector<Eigen::Vector3d> points;
    for (auto& p : astar_rst) {
      Eigen::Vector3d pt;
      pt.x() = p->x + _map_center.x();
      pt.y() = p->y + _map_center.y();
      pt.z() = p->z + _map_center.z();
      points.push_back(pt);
    }

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

      for (int j = 1; j < point_num_one_piece; ++j) {
        // Skip the first point. Which is the same as the last point on the last piece.
        float t = (float)j * _astar_planner.time_step_trajectory;

        Eigen::Vector3d p;
        p.x() = _map_center.x() + node1->x + node1->vx * t + 0.5 * ax * t * t;
        p.y() = _map_center.y() + node1->y + node1->vy * t + 0.5 * ay * t * t;
        p.z() = _map_center.z() + node1->z + node1->vz * t + 0.5 * az * t * t;

        a_star_traj_points_to_show.push_back(p);
      }
    }
    _vis->visualizeAstarPath(a_star_traj_points_to_show);

    /************************************************************/
    /***** STEP3: Risk-constrained corridor in global frame *****/
    /************************************************************/

    double            corridor_start_time = ros::Time::now().toSec();
    vector<Corridor*> corridors;
    _astar_planner.findCorridors(corridors, 2);
    double corridor_time = ros::Time::now().toSec() - corridor_start_time;
    ROS_INFO("[PLANNING]: corridor generation takes %lf s", corridor_time);

    /// Publish corridors to optimization planner
    decomp_ros_msgs::DynPolyhedronArray crd_msg;
    crd_msg.header.stamp = ros::Time::now();
    crd_msg.start_pos.x  = astar_rst[0]->x + _map_center.x();
    crd_msg.start_pos.y  = astar_rst[0]->y + _map_center.y();
    crd_msg.start_pos.z  = astar_rst[0]->z + _map_center.z();
    crd_msg.start_vel.x  = astar_rst[0]->vx;
    crd_msg.start_vel.y  = astar_rst[0]->vy;
    crd_msg.start_vel.z  = astar_rst[0]->vz;
    crd_msg.start_acc.x  = a_start.x();
    crd_msg.start_acc.y  = a_start.y();
    crd_msg.start_acc.z  = a_start.z();

    crd_msg.end_pos.x = astar_rst[astar_rst.size() - 1]->x + _map_center.x();
    crd_msg.end_pos.y = astar_rst[astar_rst.size() - 1]->y + _map_center.y();
    crd_msg.end_pos.z = astar_rst[astar_rst.size() - 1]->z + _map_center.z();
    crd_msg.end_vel.x = astar_rst[astar_rst.size() - 1]->vx;
    crd_msg.end_vel.y = astar_rst[astar_rst.size() - 1]->vy;
    crd_msg.end_vel.z = astar_rst[astar_rst.size() - 1]->vz;

    crd_msg.end_acc.x = 0.f;
    crd_msg.end_acc.y = 0.f;
    crd_msg.end_acc.z = 0.f;

    /** convert corridor (Corridor) to polyhedra (Eigen::Matrix) and publish **/
    std::vector<double>                       time_alloc;
    std::vector<Eigen::Matrix<double, 6, -1>> polyhedra;
    polyhedra.reserve(corridors.size());
    time_alloc.reserve(corridors.size());

    for (auto& corridor : corridors) {
      decomp_ros_msgs::DynPolyhedron dyn_crd;
      Eigen::MatrixXd                polygon;
      polygon.resize(6, corridor->envelope.surfaces.size());
      dyn_crd.duration = _config.a_star_search_time_step;
      time_alloc.push_back(static_cast<double>(_config.a_star_search_time_step));
      int i = 0;
      for (const auto& surface : corridor->envelope.surfaces) {
        geometry_msgs::Point point;
        geometry_msgs::Point normal;
        point.x  = surface.point.x + _map_center.x();
        point.y  = surface.point.y + _map_center.y();
        point.z  = surface.point.z + _map_center.z();
        normal.x = surface.normal.x;
        normal.y = surface.normal.y;
        normal.z = surface.normal.z;

        polygon.col(i).tail<3>() << point.x, point.y, point.z;
        polygon.col(i).head<3>() << normal.x, normal.y, normal.z;
        dyn_crd.points.push_back(point);
        dyn_crd.normals.push_back(normal);
        i++;
      }
      polyhedra.push_back(polygon);
      crd_msg.dyn_polyhedrons.push_back(dyn_crd);
    }
    _corridor_pub.publish(crd_msg);
    _vis->visualizeCorridors(polyhedra, _odom_pos);

    /**********************************************************/
    /***** STEP4: Trajectory Optimization in global frame *****/
    /**********************************************************/

    double optimization_start_time = ros::Time::now().toSec();

    /** extract init final states from the corridors **/
    Eigen::Matrix3d init_state, final_state;
    init_state.col(0) << astar_rst[0]->x + _map_center.x(), astar_rst[0]->y + _map_center.y(),
        astar_rst[0]->z + _map_center.z();
    init_state.col(1) << astar_rst[0]->vx, astar_rst[0]->vy, astar_rst[0]->vz;
    init_state.col(2) << a_start.x(), a_start.y(), a_start.z();

    int n = astar_rst.size() - 1;
    final_state.col(0) << astar_rst[n]->x + _map_center.x(), astar_rst[n]->y + _map_center.y(),
        astar_rst[n]->z + _map_center.z();
    final_state.col(1) << astar_rst[n]->vx, astar_rst[n]->vy, astar_rst[n]->vz;
    final_state.col(2) << 0.f, 0.f, 0.f;

    Eigen::Vector3d init_pos  = init_state.col(0);
    Eigen::Vector3d final_pos = final_state.col(0);
    _vis->visualizeStartGoal(init_pos);
    _vis->visualizeStartGoal(final_pos);

    /** apply optimization **/
    is_success = OptimizationInCorridors(polyhedra, time_alloc, init_state, final_state);
    // _prev_opt_end_time = ros::Time::now().toSec();
    ROS_INFO("[PLANNING] trajectory optimization takes %lf s",
             ros::Time::now().toSec() - optimization_start_time);

    if (is_success) { /* visualize trajectory */
      double v_max   = _traj.getMaxVelRate();
      _traj_duration = _traj.getDuration();
      Eigen::Vector3d zero(0, 0, 0);
      _vis->visualizeTrajectory(zero, _traj, v_max);
      _traj_end_time = _traj_start_time + ros::Duration(_traj_duration);
    }
  }  // end of if a_star valid
  _last_plan_time = ros::Time::now();

  return is_success;
}

/**
 * @brief return true when current trajectory is almost finished, drone should replan a new
 * trajectory
 *
 * @return true current trajectory is almost finished
 * @return false current trajectory is still valid
 */
bool Planner::executeTrajectory() {
  ros::Time now = ros::Time::now();
  // double    total   = _traj_end_time.toSec() - _last_plan_time.toSec();
  double          elapsed = now.toSec() - _last_plan_time.toSec();
  Eigen::Vector3d end     = _traj.getPos(_traj.getDuration());
  Eigen::Vector3d err     = end - _odom_pos;
  // if (elapsed > 0.8 * total) {
  if ((_odom_pos - end).norm() < 1.0 || elapsed > 1.0) {
    return true;
  } else {
    return false;
  }
}

bool Planner::checkTimeLapse(double time) {
  ros::Time now     = ros::Time::now();
  double    elapsed = now.toSec() - _last_plan_time.toSec();
  if (elapsed > time) {
    return true;
  } else {
    return false;
  }
}

/**
 * @brief Publish the trajectory
 *
 */
void Planner::publishTrajectory(const polynomial::Trajectory& traj) {
  _traj_idx++;
  traj_utils::PolyTraj poly_msg;
  int                  piece_num = traj.getPieceNum();
  poly_msg.drone_id              = 0;
  poly_msg.traj_id               = _traj_idx;
  poly_msg.start_time            = _traj_start_time;
  poly_msg.order                 = 7;
  poly_msg.duration.resize(piece_num);
  poly_msg.coef_x.resize(8 * piece_num);
  poly_msg.coef_y.resize(8 * piece_num);
  poly_msg.coef_z.resize(8 * piece_num);

  for (int i = 0; i < piece_num; i++) {
    poly_msg.duration[i] = traj[i].getDuration();

    Eigen::Matrix<double, 3, 8> coef = traj[i].getCoeffs();
    int                         idx  = i * 8;
    for (int j = 0; j < 8; j++) {
      poly_msg.coef_x[idx + j] = coef.row(0)[j];
      poly_msg.coef_y[idx + j] = coef.row(1)[j];
      poly_msg.coef_z[idx + j] = coef.row(2)[j];
    }
  }

  _traj_pub.publish(poly_msg);
}

/**
 * @brief
 *
 * @param traj
 * @return double maximum risk
 */
double Planner::getMaxRisk(const polynomial::Trajectory& traj) {
  double r = 0.0;
  double T = traj.getDuration();
  T        = (T > 1.0) ? T : 1.0;
  for (double t = 0.0; t < T; t += 0.1) {
    Eigen::Vector3d p     = traj.getPos(t);
    int             idx   = getPointSpatialIndexInMap(p - _map_center, _map_center);
    double          r_tmp = _future_risk[idx][0];
    // TODO ask risk map time step, use current time step by now
    if (r_tmp > r) {
      r = r_tmp;
    }
  }
  return r;
}

/**
 * @brief
 *
 * @param traj
 * @return true this trajectory is safe
 * @return false this trajectory is not safe
 */
bool Planner::checkTrajectoryRisk(const polynomial::Trajectory& traj) {
  double risk = getMaxRisk(traj);
  std::cout << termcolor::yellow << "Risk: " << risk << termcolor::reset << std::endl;
  if (risk > _config.risk_threshold_motion_primitive) {
    return false;
  } else {
    return true;
  }
}

}  // namespace planner
