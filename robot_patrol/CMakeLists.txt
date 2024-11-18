cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

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

ament_package()

# Add the patrol_class library
add_library(patrol_class src/patrol_class.cpp)

# Link the required libraries to patrol_class
ament_target_dependencies(patrol_class
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Add the patrol executable
add_executable(patrol src/patrol.cpp)

# Link the patrol_class library to the executable
target_link_libraries(patrol patrol_class)

# add_executable(patrol src/patrol.cpp src/patrol_class.cpp)
# ament_target_dependencies(patrol rclcpp sensor_msgs geometry_msgs)

install(TARGETS patrol
    DESTINATION lib/${PROJECT_NAME})

# Install executable
install(TARGETS patrol
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

