cmake_minimum_required(VERSION 3.22)

project(duatic_teleop_gamepad)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Everything the node is made of: reading the pad (input), following the robot (robot), and
# the modes it can be in (modes). The node itself and its parameter declarations live in the
# executable below, so what is here can be exercised without one.
add_library(gamepad_core
  src/input/gamepad_input.cpp
  src/modes/drive_mode.cpp
  src/modes/jog_mode.cpp
  src/modes/mode_manager.cpp
  src/robot/controller_manager_client.cpp
  src/robot/controller_snapshot.cpp
  src/robot/joint_state_cache.cpp
  src/robot/jtc_discovery.cpp
)
target_compile_features(gamepad_core PUBLIC cxx_std_20)
target_include_directories(gamepad_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(COMMAND ament_target_dependencies)
  ament_target_dependencies(gamepad_core controller_manager_msgs geometry_msgs rclcpp rcpputils sensor_msgs
    std_msgs trajectory_msgs)
else()
  target_link_libraries(gamepad_core rclcpp::rclcpp rcpputils::rcpputils ${controller_manager_msgs_TARGETS}
    ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${trajectory_msgs_TARGETS})
endif()

add_executable(gamepad_interface
  src/gamepad_config.cpp
  src/gamepad_node.cpp
  src/main.cpp
)
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

  # One binary per test file, named after the file. CONFIGURE_DEPENDS so that adding a test
  # is adding a file, without this list and the test folder being able to disagree.
  file(GLOB_RECURSE test_sources CONFIGURE_DEPENDS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} test/*.cpp)

  foreach(test_source IN LISTS test_sources)
    get_filename_component(test_name ${test_source} NAME_WE)
    ament_add_gtest(${test_name} ${test_source})
    target_link_libraries(${test_name} gamepad_core)
  endforeach()
endif()

ament_package()
