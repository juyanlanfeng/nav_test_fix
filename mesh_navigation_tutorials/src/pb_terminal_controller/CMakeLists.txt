cmake_minimum_required(VERSION 3.8)
project(pb_terminal_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mbf_mesh_core REQUIRED)
find_package(mbf_msgs REQUIRED)
# mesh_controller's exported target links lvr2::lvr2, so lvr2 must be found first.
find_package(lvr2 REQUIRED)
find_package(mesh_controller REQUIRED)
find_package(mesh_map REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Pure terminal logic (no ROS, no mesh library): the unit tests exercise this.
add_library(pb_terminal_logic src/terminal_logic.cpp)
target_include_directories(pb_terminal_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# The plugin itself; the tracking layer is the stock MeshController, used unchanged.
add_library(${PROJECT_NAME} SHARED src/pb_terminal_controller.cpp)
target_link_libraries(${PROJECT_NAME} pb_terminal_logic)

# mesh_controller installs the `include` directory *into* `include`, so its headers
# live at <prefix>/include/include while the exported target advertises
# <prefix>/include.  Add the real directory without touching that package.
get_target_property(_mesh_controller_includes mesh_controller::mesh_controller
  INTERFACE_INCLUDE_DIRECTORIES)
foreach(_dir IN LISTS _mesh_controller_includes)
  if(EXISTS "${_dir}/mesh_controller/mesh_controller.h")
    target_include_directories(${PROJECT_NAME} PRIVATE "${_dir}")
  elseif(EXISTS "${_dir}/include/mesh_controller/mesh_controller.h")
    target_include_directories(${PROJECT_NAME} PRIVATE "${_dir}/include")
  endif()
endforeach()
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs mbf_mesh_core mbf_msgs mesh_controller mesh_map pluginlib rclcpp
  std_msgs tf2_ros
)

install(TARGETS pb_terminal_logic ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
pluginlib_export_plugin_description_file(mbf_mesh_core pb_terminal_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_terminal_logic test/test_terminal_logic.cpp)
  target_link_libraries(test_terminal_logic pb_terminal_logic)
endif()

ament_export_include_directories(include)
ament_export_libraries(pb_terminal_logic)
ament_export_dependencies(geometry_msgs mbf_mesh_core mbf_msgs mesh_controller mesh_map
  pluginlib rclcpp std_msgs tf2_ros)
ament_package()
