cmake_minimum_required(VERSION 3.8)
project(path_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)

set(lib_dep_pkgs
rclcpp
nav2_costmap_2d
pluginlib)

set(sim_dep_pkgs
rclcpp
geometry_msgs)  

add_executable(RivalSim 
  src/RivalSim.cpp)

add_library(path_layer SHARED
  plugin/path_layer.cpp)

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_costmap_2d path_layer.xml)
ament_target_dependencies(path_layer ${lib_dep_pkgs})
ament_target_dependencies(RivalSim ${sim_dep_pkgs})

install(TARGETS 
  path_layer
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  RivalSim
  DESTINATION lib/${PROJECT_NAME})

ament_package()
