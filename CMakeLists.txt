cmake_minimum_required(VERSION 3.8)
project(planner_playground)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
)

set(dependencies
  ament_cmake
  action_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  geometry_msgs
  ament_index_cpp
  tf2_geometry_msgs
  std_srvs
  nav_msgs
  visualization_msgs
  interactive_markers
)

add_executable(simulator src/simulator.cpp)
ament_target_dependencies(simulator ${dependencies})
target_link_libraries(simulator yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # skip copyright and license
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS simulator RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
