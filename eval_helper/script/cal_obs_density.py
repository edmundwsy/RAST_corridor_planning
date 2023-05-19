#!/usr/bin/env python3
"""
Date: 2023-02-18
Author: Siyuan Wu 
Email: siyuanwu99@gmail.com
"""


import numpy as np
import sys
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2

map_x = map_y = map_z = resolution = 1  # map size
map_volumn = 1
num_voxels = 1
obstacle_volum = 0


def obsCallback(msg):
    obstacle_volum = 0
    max_vel = 0  # max velocity
    min_vel = 100  # min velocity
    count = 0
    for mk in msg.markers:
        count += 1
        if mk.type == 2:
            print("obstacle {} is a sphere".format(count))
            continue
        px = mk.points[0].x
        py = mk.points[0].y
        w = mk.scale.x / 2
        h = mk.points[0].z
        vx = mk.points[1].x - mk.points[0].x
        vy = mk.points[1].y - mk.points[0].y
        area = np.pi * w * w
        print("obstacle {} with width {} occupied {} area".format(count, w, area))
        obstacle_volum += area
        v = np.sqrt(vx * vx + vy * vy)
        if v > max_vel:
            max_vel = v
        if v < min_vel:
            min_vel = v

    print("obstacle count: %d  " % count)
    print("obstacle volumn: %.4f  " % obstacle_volum)
    print("obstacle density: %.6f percent " % (100 * obstacle_volum / (map_x * map_y)))
    print("max velocity: %.4f  " % max_vel)
    print("min velocity: %.4f  " % min_vel)
    rospy.signal_shutdown("Done")


def point_cloud_cb(msg):
    print(msg.header.frame_id)
    print("point cloud size: %d | %d" % (len(msg.data), num_voxels))
    print("density: %.6f percent" % (100 * len(msg.data) / num_voxels))


if __name__ == "__main__":
    rospy.init_node("cal_obstacle_density", anonymous=True)
    map_x = rospy.get_param("/map_generator/map/x_size")
    map_y = rospy.get_param("/map_generator/map/y_size")
    map_z = rospy.get_param("/map_generator/map/z_size")

    resolution = rospy.get_param("/map_generator/map/resolution")
    print("map size: %d x %d x %d" % (map_x, map_y, map_z))
    map_volumn = map_x * map_y * map_z
    num_voxels = map_x * map_y * map_z / resolution**3
    print("map volumn: %.4f  " % map_volumn)
    print("Number of voxels: %d" % (num_voxels))

    sub = rospy.Subscriber(
        # "/map_generator/global_cylinder_state", MarkerArray, obsCallback
        # "/ground_truth_state",
        "/map_generator/global_cylinder_state",
        MarkerArray,
        obsCallback,
    )
    pc_sub = rospy.Subscriber(
        "/map_generator/global_cloud", PointCloud2, point_cloud_cb
    )

    rospy.spin()
