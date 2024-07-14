cmake_minimum_required( VERSION 2.8.3)

project( quadrotor_payload_mpc )

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )
add_compile_options(-std=c++14)

# Prerequisites
find_package( ACADO REQUIRED )

# Include directories
include_directories( . ${ACADO_INCLUDE_DIRS} )


# Build an executable
add_executable( quadrotor_payload_mpc_codegen quadrotor_payload_mpc.cpp Yaml.cpp)
target_link_libraries( quadrotor_payload_mpc_codegen ${ACADO_SHARED_LIBRARIES} )
set_target_properties( quadrotor_payload_mpc_codegen PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )

add_executable( quadrotor_payload_mpc_with_ext_force_codegen quadrotor_payload_mpc_with_ext_force.cpp Yaml.cpp)
target_link_libraries( quadrotor_payload_mpc_with_ext_force_codegen ${ACADO_SHARED_LIBRARIES} )
set_target_properties( quadrotor_payload_mpc_with_ext_force_codegen PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )

add_executable( hybird_quadrotor_payload_mpc_codegen quadrotor_payload_mpc_hybird.cpp Yaml.cpp)
target_link_libraries( hybird_quadrotor_payload_mpc_codegen ${ACADO_SHARED_LIBRARIES} )
set_target_properties( hybird_quadrotor_payload_mpc_codegen PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )