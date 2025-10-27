cmake_minimum_required(VERSION 2.8.3)
project(active_perception)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
message_generation
visualization_msgs
plan_env
bspline
path_searching
cv_bridge
nav_msgs
geometry_msgs
message_filters
file_utils
time_utils
lkh_tsp_solver
)

add_message_files(
  FILES
  frontierStatistics.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES active_perception
 CATKIN_DEPENDS plan_env bspline path_searching message_filters message_generation message_runtime file_utils time_utils lkh_tsp_solver
 DEPENDS system_lib 
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library( active_perception
    src/traj_visibility.cpp 
    src/heading_planner.cpp
    src/frontier_finder.cpp
    src/graph_node.cpp
    src/perception_utils.cpp
    src/topo_graph_ikdtree.cpp
    src/graph/plan_graph_ikdtree.cpp
    src/utils/ikd_Tree.cpp
    )
add_dependencies(active_perception ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( active_perception
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    )  
