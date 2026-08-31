cmake_minimum_required(VERSION 3.22)

project(duatic_teleop_gamepad)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Logic shared by the node and its tests. Kept free of node ownership so it can be tested
# without a running ROS graph.
add_library(gamepad_core
  src/joint_state_cache.cpp
  src/robot_model.cpp
)
target_compile_features(gamepad_core PUBLIC cxx_std_20)
target_include_directories(gamepad_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(COMMAND ament_target_dependencies)
  ament_target_dependencies(gamepad_core rclcpp sensor_msgs)
else()
  target_link_libraries(gamepad_core rclcpp::rclcpp ${sensor_msgs_TARGETS})
endif()

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
endif()

ament_package()
