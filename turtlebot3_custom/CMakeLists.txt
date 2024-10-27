cmake_minimum_required(VERSION 3.8)
project(turtlebot3_custom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)  
find_package(rclpy REQUIRED)
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
# Install the worlds directory
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    src/butler_robot.py  
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip the linter for copyright checks
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint if in a non-git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

