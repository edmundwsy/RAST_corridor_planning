#!/usr/bin/env python3

"""
Date: 2022-03-02
Author: Gang Chen
Email: 947089399@qq.com
"""

import os
import time
import sys
import subprocess

""" Result parameters """
ros_info_unique_key_words_flight_time = "Flight finished"
ros_info_unique_key_words_collision_dynamic = "Current collision time"
ros_info_unique_key_words_collision_static = "Static Collision"
ros_info_log_file_path = "/home/clarence/.ros/log/latest/rosout.log"
result_save_folder = "/home/clarence/ros_ws/corridor_cost_ws/test_result/"
# result_save_folder = '/home/clarence/ros_ws/nmpc_ws/comparison/A_star/'

""" Noise parameters for the test"""
localization_noise_stddev_list = [0, 0.033, 0.066]
measurement_noise_stddev_list = [0, 0.01, 0.02]

noise_maker_launch_file_path = "/home/clarence/ros_ws/corridor_cost_ws/src/noise_maker/launch/odom_and_camera_noise.launch"

""" [Optional]: Paramter to change own parameters in Yaml """
my_yaml = (
    "/home/clarence/ros_ws/corridor_cost_ws/src/dynamic_occupancy_map/cfg/cfg.yaml"
)


""" World List """
world_folder = "/home/clarence/git/px4/master/PX4-Autopilot/Tools/sitl_gazebo/worlds/"
world_name_list = [
    "pillar_pedestrian",
    "simple_school",
]  # , 'pedestrians_dense' 'pedestrian_street2']
# world_name_list = ['simple_school', 'pillar_pedestrian']  # 'pedestrians_dense' 'pedestrian_street2',

""" The following commands should be modified according to the commands you want to run """
bash_cmd = "source /home/clarence/.bashrc"
my_ws_source_cmd = "source /home/clarence/ros_ws/corridor_cost_ws/devel/setup.bash"

gazebo_cmd1 = "cd /home/clarence/git/px4/master/PX4-Autopilot/"
gazebo_cmd2 = "DONT_RUN=1 make px4_sitl_default gazebo"
gazebo_cmd3 = "source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default"
gazebo_cmd4 = "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)"
gazebo_cmd5 = "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo"
gazebo_cmd6 = "roslaunch px4 posix_sitl.launch"

mavros_and_noise_maker_cmd = "roslaunch noise_maker odom_and_camera_noise.launch"  # will source with my_ws_source_cmd

pva_track_command = "rosrun pva_tracker tracker_sim_auto_arm_takeoff"

planning_and_evaluation_command = (
    "roslaunch dynamic_occpuancy_map planning_and_evaluation.launch"
)

# This node is the key node to check if the mav arrives at goal. If this node is not running, the process will restart.
key_planning_node_name = "planning_node"


""" Delay time: this delay time is to test more possibilities for different tests """
delay_time_list = [
    1.5,
    5.3,
    6.4,
    9.9,
    12,
    16.3,
    18.5,
    22,
    26,
    29.2,
    32.3,
    34.1,
    38.2,
    41.2,
    43.9,
    46.8,
]


def getDelayTime(seq):
    while seq >= len(delay_time_list):
        seq = seq - len(delay_time_list)

    return delay_time_list[seq]


def checkIfNodeRunning(node_name):
    pid = os.popen("pidof " + node_name).read()
    # print('pid of ' + node_name + ' is ' + pid)
    return pid


def cleanROSLog():
    os.system("gnome-terminal -- bash -c '%s; rosclean purge -y'" % bash_cmd)


def startAll(world_path, extra_delay_time=0):
    # os.system("gnome-terminal -- %s roscore --tab -- %s %s" % (bash_cmd, bash_cmd, gazebo_cmd))

    # os.system("gnome-terminal -- bash -c '%s; roscore'" % bash_cmd)
    # time.sleep(2)

    os.system(
        "gnome-terminal -- bash -c '%s; %s; %s; %s; %s; %s; %s'"
        % (
            bash_cmd,
            gazebo_cmd1,
            gazebo_cmd2,
            gazebo_cmd3,
            gazebo_cmd4,
            gazebo_cmd5,
            gazebo_cmd6 + " world:=" + world_path,
        )
    )
    time.sleep(5)
    os.system(
        "gnome-terminal -- bash -c '%s; %s; %s'"
        % (bash_cmd, my_ws_source_cmd, mavros_and_noise_maker_cmd)
    )

    time.sleep(2)
    os.system(
        "gnome-terminal -- bash -c '%s; %s; %s'"
        % (bash_cmd, my_ws_source_cmd, pva_track_command)
    )

    time.sleep(20 + extra_delay_time)
    os.system(
        "gnome-terminal -- bash -c '%s; %s; %s'"
        % (bash_cmd, my_ws_source_cmd, planning_and_evaluation_command)
    )
    time.sleep(10)


def stopNode(node_name):
    node_pid = checkIfNodeRunning(node_name)
    if node_pid:
        os.system("kill " + node_pid)
        time.sleep(1)


def stopAll():
    os.system("kill $(pgrep bash)")
    time.sleep(5)

    stopNode("gzserver")
    time.sleep(1)

    stopNode("gzclient")
    time.sleep(1)


def changeParameterInYaml(file, parameter, new_value):
    file_data = ""
    with open(file, "r") as f:
        for line in f:
            if parameter in line:
                line = "%s: %s \n" % (parameter, new_value)
            file_data += line
    with open(file, "w") as f:
        f.write(file_data)


def changeParameterInLaunch(file, parameter, new_value):
    file_data = ""
    with open(file, "r") as f:
        for line in f:
            if parameter in line:
                line = '    <param name="%s" value="%s"/> \n' % (parameter, new_value)
            file_data += line
    with open(file, "w") as f:
        f.write(file_data)


def changeNoiseLevel(measurement_noise, localization_noise):
    changeParameterInLaunch(
        noise_maker_launch_file_path, "/noise/stddev/z_1m", measurement_noise
    )
    changeParameterInLaunch(
        noise_maker_launch_file_path, "/noise/stddev/x", localization_noise
    )


if __name__ == "__main__":

    """Clean log first"""
    cleanROSLog()
    time.sleep(1)

    """ Set loop times """
    test_times_each_noise_level = 30
    max_waiting_time = 60  # Unit: second. If flight time is larger than max_waiting_time, freezing is detected.

    """ Main loop """
    for world in world_name_list:
        for measurement_noise_this in measurement_noise_stddev_list:
            for localization_noise_this in localization_noise_stddev_list:

                save_file_name = (
                    result_save_folder
                    + world
                    + "_measurement_noise_"
                    + str(measurement_noise_this)
                    + "_localization_noise_"
                    + str(localization_noise_this)
                    + ".txt"
                )

                changeNoiseLevel(measurement_noise_this, localization_noise_this)

                """ [Optional]: change a parameter in your own yaml """
                changeParameterInYaml(
                    my_yaml, "localization_stddev", localization_noise_this
                )

                with open(save_file_name, "a+") as result:
                    result.write("\n ######### A new round ########## \n")

                seq = 0
                while seq < test_times_each_noise_level:
                    with open(save_file_name, "a+") as result:
                        result.write("\n seq = " + str(seq) + "\n")

                        """ Start testing """
                        print("*** Start test: " + save_file_name + " Seq: " + str(seq))
                        delay_time = getDelayTime(seq)
                        startAll(world_folder + world + ".world", delay_time)

                        counter = 0
                        while counter < max_waiting_time:  # Wait time max: 60s
                            planning_node_pid = checkIfNodeRunning(
                                key_planning_node_name
                            )
                            if not planning_node_pid:
                                print(
                                    key_planning_node_name + " is not running. Break. "
                                )
                                break

                            counter = counter + 1
                            time.sleep(1)

                        """ Stop this test """
                        stopNode("tracker_sim_auto_arm_takeoff")
                        time.sleep(
                            5
                        )  # In case the planning time hasn't been recorded in rosout.log
                        stopAll()

                        """ Find and record data """
                        program_crashed = False
                        flight_time_line = ""
                        collision_dynamic_times_line = ""
                        collision_static_times_line = ""

                        with open(ros_info_log_file_path, "r", encoding="utf-8") as log:
                            for line in log:
                                """Flight time"""
                                if ros_info_unique_key_words_flight_time in line:
                                    flight_time_line = line
                                if ros_info_unique_key_words_collision_dynamic in line:
                                    collision_dynamic_times_line = line
                                if ros_info_unique_key_words_collision_static in line:
                                    collision_static_times_line = line

                        if flight_time_line:
                            result.write(flight_time_line)
                        elif counter == max_waiting_time:
                            result.write("MAV freezing \n")
                        else:
                            program_crashed = True

                        if program_crashed:
                            result.write("Crashed this time. \n \n")
                            print(
                                "*********** Program crashed. Invalid test. *************"
                            )
                        else:
                            result.write(collision_dynamic_times_line)
                            result.write(collision_static_times_line)
                            result.write("\n")
                            seq = seq + 1
                            print("*********** One Test finished *************")
