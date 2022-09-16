/**
 * @file multi_planning_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-07-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <ros/ros.h>

#include "planning_bezier.h"

using namespace planner;

int main(int argc, char **argv) {
  ros::init(argc, argv, "planning_bezier_node");
  ros::NodeHandle nh("~");
  BezierPlanner   b_planner(nh, PlannerConfig(nh));
  b_planner.init();
  ros::AsyncSpinner spinner(1);  // use 3 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}