cmake_minimum_required(VERSION 3.5)
project(opennav_docking)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(opennav_docking_msgs REQUIRED)
find_package(opennav_docking_core REQUIRED)

# potentially replace with nav2_common, nav2_package()
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)

include_directories(
  include
)

set(executable_name opennav_docking)
set(library_name ${executable_name}_core)

set(dependencies
  angles
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  sensor_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  yaml_cpp_vendor
  opennav_docking_msgs
  opennav_docking_core
)

add_library(${library_name} SHARED
  src/controller.cpp
  src/docking_server.cpp
  src/dock_database.cpp
  src/navigator.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_library(pose_filter SHARED
  src/pose_filter.cpp
)

add_library(nav_type_selector SHARED
  src/nav_type_selector.cpp
)

ament_target_dependencies(pose_filter
  ${dependencies}
)

ament_target_dependencies(nav_type_selector
  ${dependencies}
)

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "opennav_docking::DockingServer")

add_library(simple_charging_dock SHARED
  src/simple_charging_dock.cpp
)
ament_target_dependencies(simple_charging_dock
  ${dependencies}
)
target_link_libraries(simple_charging_dock pose_filter nav_type_selector)

pluginlib_export_plugin_description_file(opennav_docking_core plugins.xml)

install(TARGETS ${library_name} nav_type_selector pose_filter simple_charging_dock
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES test/test_dock_file.yaml
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
