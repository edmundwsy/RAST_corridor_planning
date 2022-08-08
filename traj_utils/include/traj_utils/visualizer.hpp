/**
 * @file visualizer.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief based on GCOPTER(https://github.com/ZJU-FAST-Lab/GCOPTER) from Zhepei Wang
 * @version 1.0
 * @date 2022-07-15
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <traj_utils/poly_traj.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
using namespace std;
namespace visualizer
{
  /**
   * @brief jet color map
   * to display colorful velocity on trajectories
   * @param a
   * @return Eigen::Vector3d
   */
  inline Eigen::Vector3d jetColor(double a) {
    double          s = a * 4;
    Eigen::Vector3d c;  // [r, g, b]
    switch (static_cast<int>(floor(s))) {
      case 0:
        c << 0, 0, s;
        break;
      case 1:
        c << 0, s - 1, 1;
        break;
      case 2:
        c << s - 2, 1, 3 - s;
        break;
      case 3:
        c << 1, 4 - s, 0;
        break;
      default:
        c << 1, 0, 0;
        break;
    }
    return c;
  }

  inline void displayTrajectory(const Eigen::Vector3d& start_pos, const polynomial::Trajectory& traj,
                                double max_vel, ros::Publisher& pub, const std::string& frame_id) {
    visualization_msgs::Marker traj_marker;
    traj_marker.header.frame_id    = frame_id;
    traj_marker.header.stamp       = ros::Time::now();
    traj_marker.type               = visualization_msgs::Marker::LINE_LIST;
    traj_marker.pose.orientation.w = 1.00;
    traj_marker.action             = visualization_msgs::Marker::ADD;
    traj_marker.id                 = 0;
    traj_marker.ns                 = "trajectory";
    traj_marker.color.r            = 0.00;
    traj_marker.color.g            = 0.50;
    traj_marker.color.b            = 1.00;
    traj_marker.scale.x            = 0.10;

    double          T     = 0.05;
    Eigen::Vector3d lastX = traj.getPos(0.0) + start_pos;
    for (double t = T; t < traj.getDuration(); t += T) {
      std_msgs::ColorRGBA c;
      Eigen::Vector3d     jets = jetColor(traj.getVel(t).norm() / max_vel);
      c.r                      = jets[0];
      c.g                      = jets[1];
      c.b                      = jets[2];
      c.a                      = 0.8;

      geometry_msgs::Point point;
      Eigen::Vector3d      X = traj.getPos(t) + start_pos;
      point.x                = lastX(0);
      point.y                = lastX(1);
      point.z                = lastX(2);
      traj_marker.points.push_back(point);
      traj_marker.colors.push_back(c);
      point.x = X(0);
      point.y = X(1);
      point.z = X(2);
      traj_marker.points.push_back(point);
      traj_marker.colors.push_back(c);
      lastX = X;
    }
    pub.publish(traj_marker);
  }

  class Visualizer {
   private:
    ros::NodeHandle _nh;
    ros::Publisher  _corridor_pub;
    ros::Publisher  _colorful_traj_pub;
    ros::Publisher  _astar_path_pub;
    ros::Publisher  _start_goal_pub;
    std::string     _frame_id;

   public:
    Visualizer(ros::NodeHandle& nh, std::string& frame_id) : _nh(nh), _frame_id(frame_id) {
      _corridor_pub      = _nh.advertise<visualization_msgs::MarkerArray>("vis_corridor", 1);
      _colorful_traj_pub = _nh.advertise<visualization_msgs::MarkerArray>("vis_color_traj", 1);
      _astar_path_pub    = _nh.advertise<visualization_msgs::Marker>("vis_astar_path", 1);
      _start_goal_pub    = _nh.advertise<visualization_msgs::Marker>("vis_start_goal", 1);
    }
    ~Visualizer() {}
    typedef std::shared_ptr<Visualizer> Ptr;

    /**
     * @brief publish trajectory and color the speed
     *
     * @param start_pos when planning starts
     * @param traj       trajectory to be visualized
     * @param max_vel    maximum velocity to visualize as red line
     */
    inline void visualizeTrajectory(const Eigen::Vector3d&      start_pos,
                                    const polynomial::Trajectory& traj,
                                    double                      max_vel) {
      displayTrajectory(start_pos, traj, max_vel, _colorful_traj_pub, _frame_id);
    }

    /**
     * @brief visualize corridors
     *
     * @param corridors
     * @param pose
     * @param rviz_map_center_locked true if map center is locked
     * @param clear_corridors
     */
    // void visualizeCorridors(std::vector<Corridor*>&     corridors,
    //                         geometry_msgs::PoseStamped& map_pose,
    //                         bool                        rviz_map_center_locked,
    //                         bool                        clear_corridors = false) {
    //   ROS_INFO("corridor num = %ld", corridors.size());

    //   if (clear_corridors) {
    //     corridors.clear();
    //     auto* empty_corridor = new Corridor();
    //     for (int i = 0; i < 10; ++i) {
    //       for (int j = 0; j < 8; ++j) {
    //         Point3D p;
    //         p.x = p.y = p.z = 1000.f;
    //         empty_corridor->envelope.vertexes.push_back(p);
    //       }
    //       corridors.push_back(empty_corridor);
    //     }
    //   } else {
    //     if (corridors.empty()) {
    //       ROS_INFO("Empty corridors !");
    //       return;
    //     }
    //   }

    //   visualization_msgs::MarkerArray corridor_mkrs;
    //   visualization_msgs::Marker      corridor_mkr;
    //   corridor_mkr.header.frame_id = _frame_id;
    //   corridor_mkr.header.stamp    = ros::Time::now();
    //   corridor_mkr.type            = visualization_msgs::Marker::CUBE;
    //   corridor_mkr.action          = visualization_msgs::Marker::ADD;
    //   corridor_mkr.ns              = "cubes";
    //   corridor_mkr.color.r         = 0.00;
    //   corridor_mkr.color.g         = 1.00;
    //   corridor_mkr.color.b         = 0.00;
    //   corridor_mkr.color.a         = 0.20;

    //   int id = 0;
    //   for (const auto& c : corridors) {
    //     corridor_mkr.id = id;
    //     ++id;

    //     corridor_mkr.pose.position.x = 0.0;
    //     corridor_mkr.pose.position.y = 0.0;
    //     corridor_mkr.pose.position.z = 0.0;

    //     for (int i = 0; i < 8; ++i) {
    //       corridor_mkr.pose.position.x += c->envelope.vertexes[i].x;
    //       corridor_mkr.pose.position.y += c->envelope.vertexes[i].y;
    //       corridor_mkr.pose.position.z += c->envelope.vertexes[i].z;
    //     }
    //     corridor_mkr.pose.position.x /= 8.0;
    //     corridor_mkr.pose.position.y /= 8.0;
    //     corridor_mkr.pose.position.z /= 8.0;

    //     if (!rviz_map_center_locked) {
    //       corridor_mkr.pose.position.x += map_pose.pose.position.x;
    //       corridor_mkr.pose.position.y += map_pose.pose.position.y;
    //       corridor_mkr.pose.position.z += map_pose.pose.position.z;
    //     }

    //     // Orientation
    //     Eigen::Quaternionf ori;
    //     ori.x() = ori.y() = ori.z() = 0.f;
    //     ori.w()                     = 1.f;

    //     float              angle = atan2(c->envelope.vertexes[4].y - c->envelope.vertexes[2].y,
    //                         c->envelope.vertexes[4].x - c->envelope.vertexes[2].x);
    //     Eigen::Quaternionf axis;  //= quad * q1 * quad.inverse();
    //     axis.w()                       = cos(angle / 2.f);
    //     axis.x()                       = 0.0;
    //     axis.y()                       = 0.0;
    //     axis.z()                       = sin(angle / 2.f);
    //     Eigen::Quaternionf rotated_att = ori * axis;

    //     corridor_mkr.pose.orientation.x = rotated_att.x();
    //     corridor_mkr.pose.orientation.y = rotated_att.y();
    //     corridor_mkr.pose.orientation.z = rotated_att.z();
    //     corridor_mkr.pose.orientation.w = rotated_att.w();

    //     // Width, length, height
    //     corridor_mkr.scale.x = sqrt((c->envelope.vertexes[2].x - c->envelope.vertexes[4].x) *
    //                                     (c->envelope.vertexes[2].x - c->envelope.vertexes[4].x) +
    //                                 (c->envelope.vertexes[2].y - c->envelope.vertexes[4].y) *
    //                                     (c->envelope.vertexes[2].y - c->envelope.vertexes[4].y));
    //     corridor_mkr.scale.y = sqrt((c->envelope.vertexes[2].x - c->envelope.vertexes[1].x) *
    //                                     (c->envelope.vertexes[2].x - c->envelope.vertexes[1].x) +
    //                                 (c->envelope.vertexes[2].y - c->envelope.vertexes[1].y) *
    //                                     (c->envelope.vertexes[2].y - c->envelope.vertexes[1].y));

    //     corridor_mkr.scale.z = c->envelope.vertexes[1].z - c->envelope.vertexes[0].z;
    //     corridor_mkrs.markers.push_back(corridor_mkr);
    //   }
    //   // Add some useless cubics to remove old one.
    //   for (int extra_id = id; extra_id < 50; ++extra_id) {
    //     corridor_mkr.id = id;
    //     ++id;

    //     corridor_mkr.pose.position.x = -1000;
    //     corridor_mkr.pose.position.y = -1000;
    //     corridor_mkr.pose.position.z = 1000;
    //     corridor_mkr.scale.x         = 0;
    //     corridor_mkr.scale.y         = 0;
    //     corridor_mkr.scale.z         = 0;
    //     corridor_mkrs.markers.push_back(corridor_mkr);
    //   }

    //   _corridor_pub.publish(corridor_mkrs);
    // }

    void visualizeAstarPath(vector<Eigen::Vector3d>& points,
                            int                      id,
                            float                    r,
                            float                    g,
                            float                    b,
                            float                    a,
                            float                    width,
                            int                      type = visualization_msgs::Marker::POINTS,
                            bool                     clear_path = false) {
      if (clear_path) {
        points.clear();
        Eigen::Vector3d p;
        p << -1000.f, -1000.f, -1000.f;
        for (int i = 1; i < 100; ++i) {
          points.push_back(p);
        }
      } else {
        if (points.empty()) {
          ROS_INFO("Empty path !");
          return;
        }
      }

      visualization_msgs::Marker astar_mkr;
      astar_mkr.header.frame_id = _frame_id;
      astar_mkr.header.stamp    = ros::Time::now();
      astar_mkr.type            = type;
      astar_mkr.action          = visualization_msgs::Marker::ADD;
      astar_mkr.ns              = "points_and_lines";
      astar_mkr.id              = id;
      astar_mkr.scale.x         = width;
      astar_mkr.scale.y         = width;
      astar_mkr.scale.z         = width;

      astar_mkr.color.a            = a;
      astar_mkr.color.r            = r;
      astar_mkr.color.g            = g;
      astar_mkr.color.b            = b;
      astar_mkr.pose.orientation.w = 1.0;

      for (const auto& point : points) {
        geometry_msgs::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        astar_mkr.points.push_back(p);
      }

      _astar_path_pub.publish(astar_mkr);
    }

    /**
     * @brief TODO(@siyuan): to be enabled when get to use quickhull.hpp file
     *
     * @param hPolys
     */
    // inline void visualizePolytope(const std::vector<Eigen::MatrixX4d>& hPolys) {
    //   // Due to the fact that H-representation cannot be directly visualized
    //   // We first conduct vertex enumeration of them, then apply quickhull
    //   // to obtain triangle meshs of polyhedra
    //   Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
    //   for (size_t id = 0; id < hPolys.size(); id++) {
    //     oldTris = mesh;
    //     Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
    //     geo_utils::enumerateVs(hPolys[id], vPoly);
    //     quickhull::QuickHull<double> tinyQH;
    //     const auto  polyHull  = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
    //     const auto& idxBuffer = polyHull.getIndexBuffer();
    //     int         hNum      = idxBuffer.size() / 3;
    //     curTris.resize(3, hNum * 3);
    //     for (int i = 0; i < hNum * 3; i++) {
    //       curTris.col(i) = vPoly.col(idxBuffer[i]);
    //     }
    //     mesh.resize(3, oldTris.cols() + curTris.cols());
    //     mesh.leftCols(oldTris.cols())  = oldTris;
    //     mesh.rightCols(curTris.cols()) = curTris;
    //   }
    //   // RVIZ support tris for visualization
    //   visualization_msgs::Marker meshMarker, edgeMarker;
    //   meshMarker.id                 = 0;
    //   meshMarker.header.stamp       = ros::Time::now();
    //   meshMarker.header.frame_id    = "odom";
    //   meshMarker.pose.orientation.w = 1.00;
    //   meshMarker.action             = visualization_msgs::Marker::ADD;
    //   meshMarker.type               = visualization_msgs::Marker::TRIANGLE_LIST;
    //   meshMarker.ns                 = "mesh";
    //   meshMarker.color.r            = 0.00;
    //   meshMarker.color.g            = 0.00;
    //   meshMarker.color.b            = 1.00;
    //   meshMarker.color.a            = 0.15;
    //   meshMarker.scale.x            = 1.0;
    //   meshMarker.scale.y            = 1.0;
    //   meshMarker.scale.z            = 1.0;
    //   edgeMarker         = meshMarker;
    //   edgeMarker.type    = visualization_msgs::Marker::LINE_LIST;
    //   edgeMarker.ns      = "edge";
    //   edgeMarker.color.r = 0.00;
    //   edgeMarker.color.g = 1.00;
    //   edgeMarker.color.b = 1.00;
    //   edgeMarker.color.a = 1.00;
    //   edgeMarker.scale.x = 0.02;
    //   geometry_msgs::Point point;
    //   int ptnum = mesh.cols();
    //   for (int i = 0; i < ptnum; i++) {
    //     point.x = mesh(0, i);
    //     point.y = mesh(1, i);
    //     point.z = mesh(2, i);
    //     meshMarker.points.push_back(point);
    //   }
    //   for (int i = 0; i < ptnum / 3; i++) {
    //     for (int j = 0; j < 3; j++) {
    //       point.x = mesh(0, 3 * i + j);
    //       point.y = mesh(1, 3 * i + j);
    //       point.z = mesh(2, 3 * i + j);
    //       edgeMarker.points.push_back(point);
    //       point.x = mesh(0, 3 * i + (j + 1) % 3);
    //       point.y = mesh(1, 3 * i + (j + 1) % 3);
    //       point.z = mesh(2, 3 * i + (j + 1) % 3);
    //       edgeMarker.points.push_back(point);
    //     }
    //   }
    //   meshPub.publish(meshMarker);
    //   edgePub.publish(edgeMarker);
    //   return;
    // }

    /**
     * @brief visualize start and goal points
     *
     * @param center
     * @param radius
     * @param sg
     */
    inline void visualizeStartGoal(const Eigen::Vector3d& center,
                                   const double&          radius,
                                   const int              sg) {
      visualization_msgs::Marker sphereMarkers, sphereDeleter;

      sphereMarkers.id                 = sg;
      sphereMarkers.type               = visualization_msgs::Marker::SPHERE_LIST;
      sphereMarkers.header.stamp       = ros::Time::now();
      sphereMarkers.header.frame_id    = _frame_id;
      sphereMarkers.pose.orientation.w = 1.00;
      sphereMarkers.action             = visualization_msgs::Marker::ADD;
      sphereMarkers.ns                 = "StartGoal";
      sphereMarkers.color.r            = 1.00;
      sphereMarkers.color.g            = 0.00;
      sphereMarkers.color.b            = 0.00;
      sphereMarkers.color.a            = 1.00;
      sphereMarkers.scale.x            = radius * 2.0;
      sphereMarkers.scale.y            = radius * 2.0;
      sphereMarkers.scale.z            = radius * 2.0;

      sphereDeleter        = sphereMarkers;
      sphereDeleter.action = visualization_msgs::Marker::DELETEALL;

      geometry_msgs::Point point;
      point.x = center(0);
      point.y = center(1);
      point.z = center(2);
      sphereMarkers.points.push_back(point);

      if (sg == 0) {
        _start_goal_pub.publish(sphereDeleter);
        ros::Duration(1.0e-9).sleep();
        sphereMarkers.header.stamp = ros::Time::now();
      }
      _start_goal_pub.publish(sphereMarkers);
    }
  };

}  // namespace visualizer
#endif  // VISUALIZER_HPP