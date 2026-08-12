cmake_minimum_required(VERSION 3.22)
project(coug_active_fgo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenMP REQUIRED)

# --- Parameters ---
generate_parameter_library(
  belief_state_monitor_parameters
  "params/belief_state_monitor_params.yaml"
)

generate_parameter_library(
  waypoint_nav2_parameters
  "params/waypoint_nav2_params.yaml"
)

# --- Component Library ---
ament_auto_add_library(belief_state_monitor_component SHARED NO_TARGET_LINK_LIBRARIES
  src/belief_state_monitor.cpp
)

target_compile_features(belief_state_monitor_component PUBLIC cxx_std_17)

target_link_libraries(belief_state_monitor_component
  belief_state_monitor_parameters
)

rclcpp_components_register_nodes(belief_state_monitor_component
  "coug_active_fgo::BeliefStateMonitorNode"
)

ament_auto_add_library(waypoint_nav2_component SHARED NO_TARGET_LINK_LIBRARIES
  src/waypoint_nav2.cpp
)

target_compile_features(waypoint_nav2_component PUBLIC cxx_std_17)

target_link_libraries(waypoint_nav2_component
  waypoint_nav2_parameters
)

rclcpp_components_register_nodes(waypoint_nav2_component
  "coug_active_fgo::WaypointNav2Node"
)

# --- Executables ---
ament_auto_add_executable(belief_state_monitor NO_TARGET_LINK_LIBRARIES
  src/belief_state_monitor_node.cpp
)
target_link_libraries(belief_state_monitor
  belief_state_monitor_component
)

ament_auto_add_executable(waypoint_nav2 NO_TARGET_LINK_LIBRARIES
  src/waypoint_nav2_node.cpp
)
target_link_libraries(waypoint_nav2
  waypoint_nav2_component
)

# --- Plugin Library ---
ament_auto_add_library(coug_active_fgo_critics SHARED NO_TARGET_LINK_LIBRARIES
  src/belief_state_critic.cpp
)

target_compile_features(coug_active_fgo_critics PUBLIC cxx_std_17)

target_link_libraries(coug_active_fgo_critics
  pluginlib::pluginlib
  OpenMP::OpenMP_CXX
)

# --- Export ---
pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
