cmake_minimum_required(VERSION 3.0.2)
project(impact_plan)

set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  message_generation
  message_filters
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_message_files(
  FILES
  PolyTraj.msg
  PolynomialTraj.msg
  PolynomialMatrix.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( ${PROJECT_NAME}_lib
  src/path_finder/dyn_a_star.cpp
  src/map/raycast.cpp
  src/map/grid_map.cpp
  src/alm/alm_opt.cpp
  src/alm/minco.cpp
  src/utils/visualizer.cpp
)

add_executable( ${PROJECT_NAME}_node 
  src/manager/plan_node.cpp
  src/manager/global_plan.cpp
  src/manager/plan_manager.cpp
)

add_executable(traj_server src/manager/traj_server.cpp)

add_dependencies( ${PROJECT_NAME}_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries( ${PROJECT_NAME}_node 
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(traj_server ${catkin_LIBRARIES})
