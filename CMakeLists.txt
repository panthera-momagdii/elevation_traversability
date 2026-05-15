cmake_minimum_required(VERSION 3.16)
project(elevation_traversability)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OccupancyElevation.msg"
  "srv/SaveMap.srv"
  DEPENDENCIES std_msgs nav_msgs
)

# Typesupport for linking executables in the same package.
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

set(COMMON_DEPS
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  grid_map_core
  grid_map_ros
  grid_map_msgs
  grid_map_cv
)

# ----- online node -----
add_executable(elevation_traversability_node
  src/elevation_traversability_node.cpp)
target_include_directories(elevation_traversability_node PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include)
target_link_libraries(elevation_traversability_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}")
ament_target_dependencies(elevation_traversability_node ${COMMON_DEPS})

# ----- offline node -----
add_executable(pcd_traversability_offline_node
  src/offline_pcd_traversability.cpp)
target_include_directories(pcd_traversability_offline_node PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include)
target_link_libraries(pcd_traversability_offline_node
  ${PCL_LIBRARIES}
  "${cpp_typesupport_target}")
ament_target_dependencies(pcd_traversability_offline_node ${COMMON_DEPS})

if(OpenMP_CXX_FOUND)
  target_link_libraries(pcd_traversability_offline_node OpenMP::OpenMP_CXX)
endif()

install(TARGETS
  elevation_traversability_node
  pcd_traversability_offline_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
