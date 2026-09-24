cmake_minimum_required(VERSION 3.8)
project(pb_gazebo_sim_support)

# CMake >= 4 rejects `cmake_minimum_required(VERSION <3.5)` inside the system
# ignition-gazebo6 / jsoncpp config files this package pulls in.  The repository's
# third-party PB2025 build already configures with this same escape hatch; setting
# it here keeps a plain `colcon build --packages-select pb_gazebo_sim_support`
# working without extra command-line arguments.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
set(IGN_MSGS_VER 8)
find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER 11)
find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER 6)

# Pure diagnostics logic, shared by the plugin and its unit test.
add_library(pb_diagnostics_logic INTERFACE)
target_include_directories(pb_diagnostics_logic INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pb_diagnostics_logic INTERFACE
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)

install(DIRECTORY include/
  DESTINATION include
)

# Same-step 6-DOF chassis ground truth (see src/chassis_truth_system.cc).
add_library(pb_chassis_truth_system SHARED src/chassis_truth_system.cc)
target_link_libraries(pb_chassis_truth_system
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)

# Read-only contact/actuator diagnostics, default off (see
# src/contact_diagnostics_system.cc).
add_library(pb_contact_diagnostics_system SHARED src/contact_diagnostics_system.cc)
target_link_libraries(pb_contact_diagnostics_system
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  pb_diagnostics_logic
)

add_library(pb_velocity_drive_system SHARED src/velocity_drive_system.cc)
target_link_libraries(pb_velocity_drive_system
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})

install(TARGETS pb_chassis_truth_system pb_contact_diagnostics_system pb_velocity_drive_system
  DESTINATION lib
)

# Make the plugin discoverable by Gazebo without hand-editing any path: the hook
# prepends this package's lib directory to the system plugin search path.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

if(BUILD_TESTING)
  # The engines themselves need a running simulation; behaviour is covered by the
  # ground-truth adapter tests in pb_vehicle_adapter and by the live verification
  # runs documented in README.md.  The switches and the message layout do not.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_diagnostics_logic test/test_diagnostics_logic.cc)
  target_link_libraries(test_diagnostics_logic pb_diagnostics_logic)
endif()

ament_package()
