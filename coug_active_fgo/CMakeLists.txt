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

# --- Parameters ---
generate_parameter_library(
  waypoint_nav2_parameters
  "params/waypoint_nav2_params.yaml"
)

# --- Component Library ---
ament_auto_add_library(waypoint_nav2_component SHARED src/waypoint_nav2.cpp)
target_include_directories(waypoint_nav2_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(waypoint_nav2_component
  waypoint_nav2_parameters
)

rclcpp_components_register_nodes(waypoint_nav2_component
  "coug_navigation::WaypointNav2Node"
)

# --- Executables ---
ament_auto_add_executable(waypoint_nav2 src/waypoint_nav2_node.cpp)
target_link_libraries(waypoint_nav2
  waypoint_nav2_component
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
