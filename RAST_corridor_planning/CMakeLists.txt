cmake_minimum_required(VERSION 3.0.2)
project(rast_corridor_planning)
set(CMAKE_BUILD_TYPE "Debug")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -lpthread")
# add_definitions(-std=c++17 -g -O3 -ftree-vectorize -ffast-math -march=native -lpthread)

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_filters
  sensor_msgs
  image_transport
  mav_msgs
  quadrotor_msgs
  traj_utils
  traj_opt
)

# find_package(osqp REQUIRED)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rast_corridor_planning
  CATKIN_DEPENDS roscpp traj_opt traj_utils
)


include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# link_directories(
#   lib
#   ${catkin_LIB_DIRS})

  message(WARNING "catkin ${catkin_LIB_DIRS} include ${catkin_INCLUDE_DIRS}")

# add_library(MINIMUM_SNAP_CORRIDOR
#   src/corridor_minisnap.cpp
# )

include_directories(/usr/local/include/munkres)
set(MunkresLIB
  /usr/local/lib/libmunkres.a
)

add_library(PLANNER
  src/multi_planning.cpp
)
target_link_libraries(PLANNER ${catkin_LIBRARIES})

add_executable(map_sim_example src/map_sim_example.cpp)
target_link_libraries(map_sim_example ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${MunkresLIB})

add_executable(mapping src/mapping.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${MunkresLIB})

add_executable(planning_node src/planning_node.cpp)
target_link_libraries(planning_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${MunkresLIB} )

# add_executable(planning_node_sim src/planning_node_sim.cpp)
# target_link_libraries(planning_node_sim ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${MunkresLIB} )

add_executable(multi_planning_node 
  src/multi_planning_node.cpp 
  src/multi_planning.cpp)

target_link_libraries(multi_planning_node
${catkin_LIBRARIES}
)

# add_dependencies(multi_planning_node ${catkin_EXPORTED_TARGETS})

# add_executable(traj_server src/traj_server.cpp)
# target_link_libraries(traj_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${MunkresLIB} MINIMUM_SNAP_CORRIDOR osqp::osqp)
