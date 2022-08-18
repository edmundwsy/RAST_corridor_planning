/**
 * @file visualizer.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-08-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _TRAJ_SRV_VISUALIZER_HPP
#define _TRAJ_SRV_VISUALIZER_HPP

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <memory>
#include <queue>
#include <string>
#include <traj_server/utils.hpp>
#include <traj_utils/visualizer.hpp>

#define MAX_VEL 3.0
// namespace visualizer {
/**
 * @brief visualizer trajectory server
 *
 */
class TrajSrvVisualizer {
 private:
  ros::NodeHandle _nh;
  ros::Publisher  _vis_queue_pub;
  Eigen::Vector3d _prev_pos;

 public:
  TrajSrvVisualizer(ros::NodeHandle& nh) : _nh(nh) {
    _vis_queue_pub = _nh.advertise<visualization_msgs::Marker>("vis_traj", 1);
  }
  ~TrajSrvVisualizer() {}
  typedef std::shared_ptr<TrajSrvVisualizer> Ptr;

  void visualizeTraj(std::queue<TrajPoint> queue,
                     double                max_vel  = MAX_VEL,
                     const std::string     frame_id = "world") {
    visualization_msgs::Marker vis_mk;
    vis_mk.header.frame_id    = frame_id;
    vis_mk.header.stamp       = ros::Time::now();
    vis_mk.type               = visualization_msgs::Marker::LINE_LIST;
    vis_mk.pose.orientation.w = 1.00;
    vis_mk.action             = visualization_msgs::Marker::ADD;
    vis_mk.id                 = 0;
    vis_mk.ns                 = "trajectory";
    vis_mk.color.r            = 0.0;
    vis_mk.color.g            = 1.0;
    vis_mk.color.b            = 1.0;
    vis_mk.color.a            = 1.00;
    vis_mk.scale.x            = 0.10;

    int i = 0;
    while (!queue.empty()) {
      i++;
      if (i % 10 == 1) {
        _prev_pos = queue.front().pos;
        queue.pop();
        continue;
      }
      Eigen::Vector3d pos = queue.front().pos;
      Eigen::Vector3d vel = queue.front().vel;
      queue.pop();

      Eigen::Vector3d     jets = visualizer::hotColorMap(vel.norm() / max_vel);
      std_msgs::ColorRGBA c;
      c.r = jets[0];
      c.g = jets[1];
      c.b = jets[2];

      geometry_msgs::Point p;
      p.x = _prev_pos[0];
      p.y = _prev_pos[1];
      p.z = _prev_pos[2];
      vis_mk.points.push_back(p);
      vis_mk.colors.push_back(c);
      p.x = pos[0];
      p.y = pos[1];
      p.z = pos[2];
      vis_mk.points.push_back(p);
      vis_mk.colors.push_back(c);
      _prev_pos = pos;
    }
    _vis_queue_pub.publish(vis_mk);
  }
};
// }  // namespace visualizer

#endif  // _TRAJ_SRV_VISUALIZER_HPP