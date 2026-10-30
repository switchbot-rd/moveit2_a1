cmake_minimum_required(VERSION 3.22)
project(onero_a1_servo LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()
find_package(generate_parameter_library REQUIRED)
set(dependencies control_msgs geometry_msgs moveit_core moveit_msgs moveit_ros_planning
  moveit_ros_planning_interface pluginlib rclcpp rclcpp_components realtime_tools
  sensor_msgs std_msgs std_srvs tf2_eigen trajectory_msgs)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(moveit_servo_lib_parameters config/servo_parameters.yaml)
add_library(onero_a1_servo_core SHARED
  src/collision_monitor.cpp src/collision_retreat.cpp src/servo.cpp
  src/utils/common.cpp src/utils/command.cpp)
target_include_directories(onero_a1_servo_core PRIVATE include)
target_link_libraries(onero_a1_servo_core moveit_servo_lib_parameters)
ament_target_dependencies(onero_a1_servo_core ${dependencies})

add_library(onero_a1_servo_ros SHARED src/servo_node.cpp)
target_include_directories(onero_a1_servo_ros PRIVATE include)
target_link_libraries(onero_a1_servo_ros onero_a1_servo_core moveit_servo_lib_parameters)
ament_target_dependencies(onero_a1_servo_ros ${dependencies})
rclcpp_components_register_node(onero_a1_servo_ros
  PLUGIN "moveit_servo::ServoNode" EXECUTABLE servo_node)

install(TARGETS onero_a1_servo_core onero_a1_servo_ros servo_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES LICENSE UPSTREAM DESTINATION share/${PROJECT_NAME})
ament_package()
