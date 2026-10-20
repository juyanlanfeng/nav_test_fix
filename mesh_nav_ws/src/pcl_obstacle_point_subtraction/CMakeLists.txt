cmake_minimum_required(VERSION 3.8)
project(pcl_obstacle_point_subtraction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# PCL
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(
  pcl_obstacle_point_subtraction 
  src/pcl_obstacle_point_subtraction.cpp
)

target_include_directories(pcl_obstacle_point_subtraction PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pcl_obstacle_point_subtraction PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  pcl_obstacle_point_subtraction
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "pcl_conversions"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "tf2_sensor_msgs"
)

target_link_libraries(
  pcl_obstacle_point_subtraction
  ${PCL_LIBRARIES}
)

add_executable(cloud_tf_publisher src/cloud_tf_publisher.cpp)
target_compile_features(cloud_tf_publisher PUBLIC cxx_std_17)
ament_target_dependencies(
  cloud_tf_publisher
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
)

install(TARGETS pcl_obstacle_point_subtraction cloud_tf_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
