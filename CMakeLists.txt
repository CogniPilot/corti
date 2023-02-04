cmake_minimum_required(VERSION 3.8)
project(corti)

# see: https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(synapse_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(bezier6
    src/casadi/bezier6.c
    )
target_compile_options(bezier6 PRIVATE -Wno-unused)

add_executable(rover_planner
    src/rover_planner.cpp
    )

ament_target_dependencies(rover_planner rclcpp synapse_msgs nav_msgs geometry_msgs)
target_link_libraries(rover_planner bezier6)

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

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

install(TARGETS
  rover_planner
  DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/rover_planner.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
