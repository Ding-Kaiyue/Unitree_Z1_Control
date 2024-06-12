cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp_action REQUIRED) 
find_package(actionlib_msgs REQUIRED) 
find_package(control_msgs REQUIRED) 
find_package(rclcpp REQUIRED) 
find_package(std_msgs REQUIRED) 
find_package(robot_interfaces REQUIRED) 

include_directories(
  ${PROJECT_SOURCE_DIR}  
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
add_executable(robot_control_node src/robot_control.cpp)
ament_target_dependencies(robot_control_node rclcpp std_msgs sensor_msgs robot_interfaces rclcpp_action actionlib_msgs control_msgs)

install(TARGETS
  robot_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
