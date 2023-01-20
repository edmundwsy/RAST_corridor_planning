#!/usr/bin/env python3
"""
Date: 2023-01-18
Author: Siyuan Wu 
Email: siyuanwu99@gmail.com
"""

import numpy as np
import sys
import os
import re
import matplotlib.pyplot as plt

file_root = "/home/siyuan/.ros/log/latest/"
file_name = "multi_eval-1-stdout.log"

file_path = os.path.join(file_root, file_name)


def read_file(path, data):
    with open(path, "r") as f:
        for line in f.readlines():
            if "[uav" in line:
                id = int(re.search(r"uav(\d)", line).group(1))
                t = re.search(r"t: (\d+\.?\d*),", line).group(1)
                if float(t) == 0:
                    continue

                pos_group = re.search(
                    r"pos: ([-+]?\d+\.?\d*), ([-+]?\d+\.?\d*), ([-+]?\d+\.?\d*)",
                    line,
                )
                vel_group = re.search(
                    r"vel: ([-+]?\d+\.?\d*), ([-+]?\d+\.?\d*), ([-+]?\d+\.?\d*)",
                    line,
                )
                acc_group = re.search(
                    r"acc: ([-+]?\d+\.?\d*), ([-+]?\d+\.?\d*), ([-+]?\d+\.?\d*)",
                    line,
                )
                yaw = re.search(r"yaw: ([-+]?\d+\.?\d*),", line).group(1)
                yaw_rate = re.search(r"yaw_rate: ([-+]?\d+\.?\d*)", line).group(1)
                data[id].append(
                    np.array(
                        [
                            float(t),
                            float(pos_group.group(1)),
                            float(pos_group.group(2)),
                            float(pos_group.group(3)),
                            float(vel_group.group(1)),
                            float(vel_group.group(2)),
                            float(vel_group.group(3)),
                            float(acc_group.group(1)),
                            float(acc_group.group(2)),
                            float(acc_group.group(3)),
                            float(yaw),
                            float(yaw_rate),
                        ]
                    )
                )


def get_min_distance():
    pass


def get_sum_control_efforts(d):
    a = d[:, 7:10]
    t = d[:, 0]
    dt = np.diff(t).reshape(-1, 1) + 1e-6
    d = np.diff(a, axis=0) / dt
    c = np.square(d).sum(axis=1).dot(dt)
    return np.sum(c)


def get_avg_flight_time(d):
    v = np.square(d[:, 4:7]).sum(axis=1)
    t0 = d[0, 0]
    i_f = len(v) - (np.flip(v) != 0).argmax()
    tf = d[i_f, 0]
    return tf - t0


def plot_x_t(data, idx):
    t = data[:, 0]
    x = data[:, idx]
    plt.plot(t, x)
    plt.show()


def plot_v_t(data):
    t = data[:, 0]
    vx = data[:, 4]
    vy = data[:, 5]
    vz = data[:, 6]
    v = np.sqrt(vx**2 + vy**2 + vz**2)
    plt.plot(t, v)
    plt.show()


def plot_a_t(data):
    t = data[:, 0]
    ax = data[:, 7]
    ay = data[:, 8]
    az = data[:, 9]
    a = np.sqrt(ax**2 + ay**2 + az**2)
    plt.plot(t, a)
    plt.show()


def plot_y_t(data):
    t = data[:, 0]
    y = data[:, 10] * 180 / np.pi
    y += [360 if i < 0 else 0 for i in y]
    plt.plot(t, y)
    plt.show()


def get_data(n, file_path):
    """
    get data from file

    :param n int: number of agents
    :param file_path [TODO:type]: [TODO:description]
    """

    data = [[] for i in range(n)]
    read_file(file_path, data)
    for (i, d) in enumerate(data):
        data_np = np.array(d)
        data[i] = data_np
    return data


if __name__ == "__main__":
    num_agents = 4
    data = [[] for _ in range(num_agents)]

    read_file(file_path, data)
    for (i, d) in enumerate(data):
        data_np = np.array(d)
        data[i] = data_np

    # plot_x_t(data[1], 1)
    # plot_a_t(data[1])
    plot_v_t(data[1])
    # plot_y_t(data[1])
    print("sum control efforts:", get_sum_control_efforts(data[1]))
    print("avg flight time:", get_avg_flight_time(data[1]))
