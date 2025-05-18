cmake_minimum_required(VERSION 3.8)
project(navigation2_run)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(opennav_docking_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(odometry_sim src/odometry_sim.cpp)
ament_target_dependencies(odometry_sim rclcpp geometry_msgs nav2_msgs tf2_ros tf2 tf2_geometry_msgs)

add_executable(script_sim src/script_sim.cpp)
ament_target_dependencies(script_sim rclcpp geometry_msgs nav2_msgs tf2_ros tf2 tf2_geometry_msgs yaml-cpp opennav_docking_msgs std_msgs)

add_executable(final_pose_bridge src/final_pose_bridge.cpp)
ament_target_dependencies(final_pose_bridge rclcpp geometry_msgs nav2_msgs)

add_executable(system_check src/system_check.cpp)
ament_target_dependencies(system_check rclcpp  std_msgs  nav2_msgs opennav_docking_msgs  btcpp_ros2_interfaces rclcpp_action)

add_executable(system_check_test_server src/system_check_test_server.cpp)
ament_target_dependencies(system_check_test_server rclcpp btcpp_ros2_interfaces)

target_link_libraries(script_sim yaml-cpp)

install(TARGETS
  odometry_sim
  script_sim
  final_pose_bridge
  system_check
  system_check_test_server
  DESTINATION lib/${PROJECT_NAME}
)

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

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_package()
