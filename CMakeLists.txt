cmake_minimum_required(VERSION 3.8)
project(rv2_server_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rv2_interfaces REQUIRED)
find_package(rv2_control_signal_transport REQUIRED)

find_package(unitree_api REQUIRED)

# ── control_server (composable node) ─────────────────────────────────────────
add_library(control_server_component SHARED
  src/control_server.cpp
  thirdparty/unitree_ros2_client/ros2_b2_sport_client.cpp
)
target_include_directories(control_server_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/unitree_ros2_client
)
ament_target_dependencies(control_server_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  rv2_interfaces
  unitree_api
  rv2_control_signal_transport
)
rclcpp_components_register_node(control_server_component
  PLUGIN "ControlServerNode"
  EXECUTABLE control_server
)
install(TARGETS control_server_component control_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ── fake_unitree_api (composable node, lives in test/) ───────────────────────
add_library(fake_unitree_api_component SHARED test/fake_unitree_api_node.cpp)
ament_target_dependencies(fake_unitree_api_component
  rclcpp
  rclcpp_components
  unitree_api
)
rclcpp_components_register_node(fake_unitree_api_component
  PLUGIN "FakeUnitreeApiNode"
  EXECUTABLE fake_unitree_api_node
)
install(TARGETS fake_unitree_api_component fake_unitree_api_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ── test_control_server (plain executable) ────────────────────────────────────
add_executable(test_control_server test/test_control_server.cpp)
target_include_directories(test_control_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(test_control_server
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  rv2_interfaces
  rv2_control_signal_transport
)
install(TARGETS test_control_server DESTINATION lib/${PROJECT_NAME})

# ── public headers ────────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp rclcpp_components
  sensor_msgs geometry_msgs std_msgs
  rv2_interfaces
  unitree_api
  rv2_control_signal_transport)

# ── launch files ──────────────────────────────────────────────────────────────
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

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
