cmake_minimum_required(VERSION 2.8.3)
project(traj_opt)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(osqp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    visualization_msgs
    plan_env
    cv_bridge
    traj_utils
    path_searching
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES traj_opt
    CATKIN_DEPENDS plan_env

    # DEPENDS system_lib
)

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library(traj_opt
    src/poly_traj_optimizer.cpp
)

target_link_libraries(traj_opt
    ${catkin_LIBRARIES}
)

add_executable(test_minisnap src/test_minisnap.cpp)
target_link_libraries(test_minisnap
    ${catkin_LIBRARIES}
    osqp::osqp
)