cmake_minimum_required(VERSION 3.8)
project(coug_active_fgo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(generate_parameter_library REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenMP REQUIRED)

# --- Parameters ---
generate_parameter_library(
  waypoint_nav2_parameters
  "params/waypoint_nav2_params.yaml"
)

generate_parameter_library(
  belief_state_monitor_parameters
  "params/belief_state_monitor_params.yaml"
)

# --- Component Libraries ---
ament_auto_add_library(waypoint_nav2_component SHARED src/waypoint_nav2.cpp)
target_include_directories(waypoint_nav2_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(waypoint_nav2_component
  waypoint_nav2_parameters
)

ament_auto_add_library(belief_state_monitor_component SHARED src/belief_state_monitor.cpp)
target_include_directories(belief_state_monitor_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(belief_state_monitor_component
  belief_state_monitor_parameters
)

# --- Executables ---
ament_auto_add_executable(waypoint_nav2 src/waypoint_nav2_node.cpp)
target_link_libraries(waypoint_nav2
  waypoint_nav2_component
)

ament_auto_add_executable(belief_state_monitor src/belief_state_monitor_node.cpp)
target_link_libraries(belief_state_monitor
  belief_state_monitor_component
)

# --- Critic Plugin ---
ament_auto_add_library(coug_active_fgo_critics SHARED
  src/belief_state_critic.cpp
)
target_include_directories(coug_active_fgo_critics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(coug_active_fgo_critics
  pluginlib::pluginlib
  OpenMP::OpenMP_CXX
)
pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)

# --- Testing ---
if(BUILD_TESTING)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  rviz
  plotjuggler
  USE_SCOPED_HEADER_INSTALL_DIR
)
