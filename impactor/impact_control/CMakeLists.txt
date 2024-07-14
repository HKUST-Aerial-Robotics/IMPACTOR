cmake_minimum_required(VERSION 3.0.2)
project(impact_control)

# ARM NEON flags
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
  message("enabling ARM neon optimizations")
endif()

# flags for speed (should already be enabled by default)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3 -march=native")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  uav_utils

)
find_package(Ceres REQUIRED)

catkin_package()

add_executable(mpc_controller_node 
  src/mpc_controller_node.cpp
  src/mpc_controller.cpp
  src/mpc_input.cpp
  src/mpc_fsm.cpp
  src/mpc_wrapper.cpp
  # MPC solver
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/quadrotor_payload_mpc/acado_qpoases_interface.cpp
  model/quadrotor_payload_mpc/acado_integrator.c
  model/quadrotor_payload_mpc/acado_solver.c
  model/quadrotor_payload_mpc/acado_auxiliary_functions.c
  )
target_include_directories(mpc_controller_node PUBLIC
  include/payload_mpc_controller
  include 
  ${EIGEN3_INCLUDE_DIR}
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  # MPC solver
  model/quadrotor_payload_mpc/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC
  ${CERES_INCLUDE_DIRS}
  )
target_link_libraries(mpc_controller_node
  # mpc_solver
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)
