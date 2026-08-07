cmake_minimum_required(VERSION 3.8)
project(robot_planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(robot_planners_core SHARED
  src/astar_planner.cpp
  src/dijkstra_planner.cpp
)

target_include_directories(robot_planners_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(robot_planners_core
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav2_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  tf2_ros
  visualization_msgs
)

# Unit tests for the grid search core (no ROS runtime needed).
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid_search test/test_grid_search.cpp)
  target_include_directories(test_grid_search PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_grid_search robot_planners_core)
endif()

install(TARGETS robot_planners_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(FILES robot_planners_plugins.xml
  DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(nav2_core robot_planners_plugins.xml)

ament_package()