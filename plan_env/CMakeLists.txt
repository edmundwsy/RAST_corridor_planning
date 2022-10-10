cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    visualization_msgs
    cv_bridge
    message_filters
    quadrotor_msgs  # only for examples
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES plan_env
    CATKIN_DEPENDS roscpp std_msgs
    DEPENDS OpenCV

    # DEPENDS system_lib
)

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
include_directories(/usr/local/include/munkres)
set(MunkresLIB
  /usr/local/lib/libmunkres.a
)


add_library(plan_env
    src/grid_map.cpp
    src/raycast.cpp
    src/obj_predictor.cpp
    src/dsp_map_new.cpp
)
target_link_libraries(plan_env
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${MunkresLIB}
)

add_executable(obj_generator
    src/obj_generator.cpp
)
target_link_libraries(obj_generator
    ${catkin_LIBRARIES}
)

add_executable(test_grid_map_node
    src/test_grid_map_node.cpp
)
target_link_libraries(test_grid_map_node
    plan_env
    ${catkin_LIBRARIES}
)

add_executable(test_dsp_map_node
    src/test_dsp_map_node.cpp
)
target_link_libraries(test_dsp_map_node
    plan_env
    ${catkin_LIBRARIES}
)

add_executable(dsp_map_node
    src/dsp_map_node.cpp
)
target_link_libraries(dsp_map_node
    plan_env
    ${catkin_LIBRARIES}
)