cmake_minimum_required(VERSION 3.8)
project(nodding_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(servo_node src/servo_node.cpp)
target_include_directories(servo_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(servo_node rclcpp sensor_msgs geometry_msgs tf2_ros)

add_executable(serial_angle_bridge src/serial_angle_bridge.cpp)
target_include_directories(serial_angle_bridge PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(serial_angle_bridge rclcpp sensor_msgs)

add_executable(servo_tf_node src/servo_tf_node.cpp)
target_include_directories(servo_tf_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(servo_tf_node rclcpp sensor_msgs geometry_msgs tf2_ros)

add_executable(scan_to_cloud src/scan_to_cloud_node.cpp)
target_include_directories(scan_to_cloud PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(scan_to_cloud
  rclcpp builtin_interfaces sensor_msgs geometry_msgs laser_geometry tf2 tf2_ros tf2_sensor_msgs)

install(TARGETS servo_node serial_angle_bridge servo_tf_node scan_to_cloud
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_geometry test/test_geometry.cpp)
  target_include_directories(test_geometry PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  add_test(NAME test_geometry COMMAND test_geometry)

  add_executable(test_laser_projection test/test_laser_projection.cpp)
  ament_target_dependencies(test_laser_projection
    geometry_msgs laser_geometry sensor_msgs tf2)
  add_test(NAME test_laser_projection COMMAND test_laser_projection)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
