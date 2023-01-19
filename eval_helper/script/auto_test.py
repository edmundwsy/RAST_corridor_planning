#!/usr/bin/env python3

"""
Date: 2022-03-02
Author: Gang Chen
Email: 947089399@qq.com
"""

import os
import time
import sys
import re
import subprocess
import argparse
import csv
import numpy as np

import calculate

parser = argparse.ArgumentParser()
parser.add_argument("--world", type=str, default="world", help="world name")
parser.add_argument("--num_agents", type=int, default="4", help="number of agents")
parser.add_argument("--save_path", type=str, default="results", help="folder name")
parser.add_argument("--waiting_time", type=float, default="20", help="waiting time")
parser.add_argument(
    "--node_name", type=str, default="/uav0/planner", help="key planning node name"
)

parser.add_argument(
    "--cmd_source",
    type=str,
    default="source /home/siyuan/workspace/thesis_workspace/devel/setup.zsh",
)
parser.add_argument(
    "--cmd_recorder",
    type=str,
    default="roslaunch eval_helper eval.launch",
)
parser.add_argument(
    "--cmd_launch",
    type=str,
    default="roslaunch plan_manager sim_baseline_fkpcp_4.launch",
)
parser.add_argument("--cmd_trigger", type=str, default="rosrun eval_helper trigger")


def startPlanner():
    pass


def checkIfNodeRunning(node_name):
    pid = os.popen("pidof " + node_name).read()
    # print('pid of ' + node_name + ' is ' + pid)
    return pid


def cleanROSLog():
    os.system("gnome-terminal -- bash -c '%s; rosclean purge -y'" % "source ~/.bashrc")


def stopNode(node_name):
    node_pid = checkIfNodeRunning(node_name)
    if node_pid:
        os.system("kill " + node_pid)
        time.sleep(1)


def startAll(arg):
    # Start the planner
    os.system("gnome-terminal -- bash -c '%s; %s;'" % (arg.cmd_source, arg.cmd_launch))
    time.sleep(1)
    # Start the recorder
    os.system(
        "gnome-terminal -- bash -c '%s; %s;'" % (arg.cmd_source, arg.cmd_recorder)
    )
    time.sleep(1)
    # Start planning
    os.system("gnome-terminal -- bash -c '%s; %s;'" % (arg.cmd_source, arg.cmd_trigger))


def stopAll():
    os.system("kill $(pgrep bash)")
    time.sleep(2)

    # stopNode("gzserver")
    # time.sleep(1)
    #
    # stopNode("gzclient")
    # time.sleep(1)


def findAndRecord(args, save_path):
    log_root = "/home/siyuan/.ros/log/latest/"

    eval_log_path = None
    for file in os.listdir(log_root):
        if re.match(r".*multi_eval.*", file):
            eval_log_path = log_root + file

    if eval_log_path is None:
        print("No evaluation log found!")
        return

    # extract data from file:
    data = calculate.get_data(args.num_agents, eval_log_path)

    ctrl_efforts = np.array([calculate.get_sum_control_efforts(d) for d in data])
    flight_times = np.array([calculate.get_avg_flight_time(d) for d in data])

    arr_to_write = np.hstack([ctrl_efforts, flight_times])
    np.savetxt(save_path, arr_to_write, fmt="%f", delimiter=",")
    print("Data saved to " + save_path)
    return


def run(arg):
    save_path = arg.save_path + arg.world + "_measurement_noise_" + ".csv"
    cleanROSLog()
    time.sleep(0.5)
    with open(save_path, "a+") as result:
        writer = csv.writer(result)
        ctr_eft_headers = ["ctrl_effort_uav" + str(i) for i in range(arg.num_agents)]
        flt_tim_headers = ["flight_time_uav" + str(i) for i in range(arg.num_agents)]
        headers = ctr_eft_headers + flt_tim_headers
        writer.writerow(headers)

    startAll(arg)

    counter = 0
    while counter < arg.waiting_time:  # Wait time max: 60s
        num_closed = 0
        for i in range(arg.num_agents):
            node_name = re.sub(r"\d", str(i), arg.node_name)
            if checkIfNodeRunning(node_name):
                num_closed += 1

        if num_closed == arg.num_agents:
            print("All agents are not running. Break. ")
            break

        counter = counter + 1
        time.sleep(1)

    " Stop "
    stopNode("node_name")
    time.sleep(1)
    stopAll()

    findAndRecord(args, save_path)


if __name__ == "__main__":
    args = parser.parse_args()
    run(args)
    # print(findAndRecord(args))
