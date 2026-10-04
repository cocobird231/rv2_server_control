cmake_minimum_required(VERSION 3.8)
project(rv2_server_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(r1_interfaces REQUIRED)
find_package(rv2_control_signal_transport REQUIRED)
find_package(joy_interpreter REQUIRED)
find_package(unitree_api REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(control_server_component SHARED
  src/control_server.cpp
  thirdparty/unitree_ros2_client/ros2_b2_sport_client.cpp)
target_include_directories(control_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/unitree_ros2_client>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}/thirdparty>)
ament_target_dependencies(control_server_component
  rclcpp rclcpp_components sensor_msgs geometry_msgs r1_interfaces
  rv2_control_signal_transport joy_interpreter unitree_api)
target_link_libraries(control_server_component
  joy_interpreter::joy_interpreter_lib nlohmann_json::nlohmann_json)
rclcpp_components_register_node(control_server_component
  PLUGIN "ControlServerNode" EXECUTABLE control_server)
install(TARGETS control_server_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS control_server RUNTIME DESTINATION lib/${PROJECT_NAME})

add_library(fake_unitree_api_component SHARED test/fake_unitree_api_node.cpp)
ament_target_dependencies(fake_unitree_api_component rclcpp rclcpp_components unitree_api)
rclcpp_components_register_node(fake_unitree_api_component
  PLUGIN "FakeUnitreeApiNode" EXECUTABLE fake_unitree_api_node)
install(TARGETS fake_unitree_api_component fake_unitree_api_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(FILES thirdparty/unitree_ros2_client/ros2_b2_sport_client.h
  thirdparty/unitree_ros2_client/patch.hpp
  DESTINATION include/${PROJECT_NAME}/thirdparty)
ament_export_include_directories(include include/${PROJECT_NAME}/thirdparty)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(control_server_component)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs geometry_msgs r1_interfaces
  rv2_control_signal_transport joy_interpreter unitree_api nlohmann_json)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES thirdparty/unitree/LICENSE
  DESTINATION share/${PROJECT_NAME}/thirdparty/unitree)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(test_unitree_build
    test/unit/test_unitree_build.py TIMEOUT 60)
  set_property(TEST test_unitree_build APPEND PROPERTY LABELS unit)
  ament_add_pytest_test(test_unitree_discovery
    test/integration/test_unitree_discovery.py TIMEOUT 90)
  set_property(TEST test_unitree_discovery APPEND PROPERTY LABELS integration)
  foreach(suite unit integration)
    ament_add_gtest(test_control_server_${suite}
      test/${suite}/test_control_server.cpp TIMEOUT 180)
    target_include_directories(test_control_server_${suite} PRIVATE test)
    target_link_libraries(test_control_server_${suite} control_server_component)
    set_property(TEST test_control_server_${suite} APPEND PROPERTY LABELS ${suite})
  endforeach()
  add_launch_test(test/integration/test_joy_bridge.py
    TARGET test_joy_bridge TIMEOUT 120)
  set_property(TEST test_joy_bridge APPEND PROPERTY LABELS integration)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/test/ament_flake8.ini")
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
