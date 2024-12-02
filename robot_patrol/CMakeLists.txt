cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Libraries
add_library(patrol_class src/patrol_class.cpp)
add_library(direction_service_class src/direction_service_class.cpp)
add_library(test_service_class src/test_service_class.cpp)
add_library(patrol_with_service_class src/patrol_with_service_class.cpp)
add_library(go_to_pose_action_class src/go_to_pose_action_class.cpp)

install(TARGETS patrol_class direction_service_class test_service_class patrol_with_service_class go_to_pose_action_class
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Link dependencies to libraries
ament_target_dependencies(patrol_class rclcpp sensor_msgs geometry_msgs)
ament_target_dependencies(direction_service_class rclcpp sensor_msgs geometry_msgs)
ament_target_dependencies(test_service_class rclcpp sensor_msgs geometry_msgs)
ament_target_dependencies(patrol_with_service_class rclcpp sensor_msgs geometry_msgs)
ament_target_dependencies(go_to_pose_action_class rclcpp sensor_msgs geometry_msgs nav_msgs rclcpp_action tf2 tf2_geometry_msgs)

# Executables
add_executable(patrol src/patrol.cpp)
add_executable(direction_service src/direction_service.cpp)
add_executable(test_service src/test_service.cpp)
add_executable(patrol_with_service src/patrol_with_service.cpp)
add_executable(go_to_pose_action src/go_to_pose_action.cpp)

# ament_target_dependencies(go_to_pose_action rclcpp sensor_msgs geometry_msgs nav_msgs rclcpp_action tf2 tf2_geometry_msgs)

# Link libraries to executables
target_link_libraries(patrol patrol_class)
target_link_libraries(direction_service direction_service_class)
target_link_libraries(test_service test_service_class)
target_link_libraries(patrol_with_service patrol_with_service_class)
target_link_libraries(go_to_pose_action go_to_pose_action_class)

# Generate interfaces for the .srv file
rosidl_generate_interfaces(${PROJECT_NAME} "srv/GetDirection.srv" "action/GoToPose.action" DEPENDENCIES std_msgs sensor_msgs geometry_msgs)
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(direction_service_class ${typesupport_target})
target_link_libraries(test_service_class ${typesupport_target})
target_link_libraries(patrol_with_service_class ${typesupport_target})
target_link_libraries(go_to_pose_action_class ${typesupport_target})
# target_link_libraries(go_to_pose_action ${typesupport_target})

# Install additional files
install(TARGETS patrol direction_service test_service patrol_with_service go_to_pose_action
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional files
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}/)

ament_package()