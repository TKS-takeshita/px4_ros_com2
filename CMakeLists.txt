cmake_minimum_required(VERSION 3.5)
project(px4_ros_com LANGUAGES C CXX CUDA)
enable_language(CUDA)
# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()

# 61 : PCのGPU 87 : Jetson Orin nanoのGPU
set(CMAKE_CUDA_ARCHITECTURES 61 87)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(CUDA)
find_package(Curses REQUIRED)


#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_executable(sensor_combined_listener src/examples/listeners/sensor_combined_listener.cpp)
ament_target_dependencies(sensor_combined_listener rclcpp px4_msgs)
install(TARGETS sensor_combined_listener DESTINATION lib/${PROJECT_NAME})

add_executable(vehicle_gps_position_listener src/examples/listeners/vehicle_gps_position_listener.cpp)
ament_target_dependencies(vehicle_gps_position_listener rclcpp px4_msgs)
install(TARGETS vehicle_gps_position_listener DESTINATION lib/${PROJECT_NAME})

add_executable(debug_vect_advertiser src/examples/advertisers/debug_vect_advertiser.cpp)
ament_target_dependencies(debug_vect_advertiser rclcpp px4_msgs)
install(TARGETS debug_vect_advertiser DESTINATION lib/${PROJECT_NAME})

add_executable(offboard_control src/examples/offboard/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs sensor_msgs geometry_msgs)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

add_executable(vehicle_odometry src/examples/offboard/vehicle_odometry.cpp)
ament_target_dependencies(vehicle_odometry rclcpp px4_msgs nav_msgs Eigen3)
target_link_libraries(vehicle_odometry frame_transforms)
install(TARGETS vehicle_odometry DESTINATION lib/${PROJECT_NAME})

##########################
# MCMPC core (CUDA)
##########################

add_library(mcmpc_core SHARED
  src/examples/offboard/mcmpc_controller.cu
  src/examples/offboard/const_params.cu
)

set_target_properties(mcmpc_core PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(mcmpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mcmpc_core
  ${CUDA_LIBRARIES}
  curand
  Eigen3::Eigen
)

ament_target_dependencies(mcmpc_core
  rclcpp
  px4_msgs
  sensor_msgs
  nav_msgs
  Eigen3
)

##########################
# quad_mcmpc_node
##########################

add_executable(quad_mcmpc_node
  src/examples/offboard/quad_mcmpc.cu
)

set_target_properties(quad_mcmpc_node PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)

target_include_directories(quad_mcmpc_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(quad_mcmpc_node
  mcmpc_core
)

ament_target_dependencies(quad_mcmpc_node
  rclcpp
  px4_msgs
  sensor_msgs
  nav_msgs
  Eigen3
)

install(TARGETS quad_mcmpc_node DESTINATION lib/${PROJECT_NAME})

##########################
# Install
##########################

ament_export_dependencies(
  ament_cmake
  rclcpp
  rosidl_default_runtime
  eigen3_cmake_module
  Eigen3
  px4_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(frame_transforms mcmpc_core)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS frame_transforms
  EXPORT export_frame_transforms
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS mcmpc_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)

##########################
# Testing
##########################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

##########################
# Python
##########################

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/examples/offboard_py/offboard_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()