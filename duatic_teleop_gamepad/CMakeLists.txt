cmake_minimum_required(VERSION 3.22)

project(duatic_teleop_gamepad)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Logic shared by the node and its tests. Kept free of node ownership so it can be tested
# without a running ROS graph.
add_library(gamepad_core
  src/controller_manager_client.cpp
  src/controller_snapshot.cpp
  src/drive_ramp.cpp
  src/gamepad_config.cpp
  src/gamepad_node.cpp
  src/gripper_toggle.cpp
  src/jog_group.cpp
  src/joint_state_cache.cpp
  src/jtc_discovery.cpp
  src/robot_model.cpp
  src/stick_mapping.cpp
  src/teleop_mode.cpp
)
target_compile_features(gamepad_core PUBLIC cxx_std_20)
target_include_directories(gamepad_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(COMMAND ament_target_dependencies)
  ament_target_dependencies(gamepad_core controller_manager_msgs geometry_msgs rclcpp sensor_msgs std_msgs
    trajectory_msgs)
else()
  target_link_libraries(gamepad_core rclcpp::rclcpp ${controller_manager_msgs_TARGETS} ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${trajectory_msgs_TARGETS})
endif()

add_executable(gamepad_interface src/main.cpp)
target_link_libraries(gamepad_interface gamepad_core)

install(TARGETS gamepad_interface
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS gamepad_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_joint_state_cache test/test_joint_state_cache.cpp)
  target_link_libraries(test_joint_state_cache gamepad_core)

  ament_add_gtest(test_robot_model test/test_robot_model.cpp)
  target_link_libraries(test_robot_model gamepad_core)

  ament_add_gtest(test_controller_snapshot test/test_controller_snapshot.cpp)
  target_link_libraries(test_controller_snapshot gamepad_core)

  ament_add_gtest(test_jtc_discovery test/test_jtc_discovery.cpp)
  target_link_libraries(test_jtc_discovery gamepad_core)

  ament_add_gtest(test_jog_group test/test_jog_group.cpp)
  target_link_libraries(test_jog_group gamepad_core)

  ament_add_gtest(test_stick_mapping test/test_stick_mapping.cpp)
  target_link_libraries(test_stick_mapping gamepad_core)

  ament_add_gtest(test_teleop_mode test/test_teleop_mode.cpp)
  target_link_libraries(test_teleop_mode gamepad_core)

  ament_add_gtest(test_drive_ramp test/test_drive_ramp.cpp)
  target_link_libraries(test_drive_ramp gamepad_core)

  ament_add_gtest(test_gripper_toggle test/test_gripper_toggle.cpp)
  target_link_libraries(test_gripper_toggle gamepad_core)
endif()

ament_package()
