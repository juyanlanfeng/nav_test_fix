cmake_minimum_required(VERSION 3.8)
project(mesh_navigation_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# install
install(DIRECTORY config launch rviz maps
  DESTINATION share/${PROJECT_NAME}
)

# The workspace-level map inspection entry point is kept in the repository's
# launch/ directory and installed under this package for `ros2 launch`.
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../launch/meshnav_map_test.launch.py"
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS scripts/meshnav_plan_only.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME meshnav_plan_only
)
install(PROGRAMS scripts/meshnav_source_cloud.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME meshnav_source_cloud
)

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
