cmake_minimum_required(VERSION 3.8)
project(drone_action_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#colcon build --packages-select drone_flight_plan_control

# find dependencies (packages)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# find dependencies (messages)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

#add executables
add_executable(state_reporter_node src/state_reporter_node.cpp)
add_executable(motion_planner_node src/motion_planner_node.cpp)
add_executable(kinematics_node src/kinematics_node.cpp)

ament_target_dependencies(state_reporter_node rclcpp geometry_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(motion_planner_node rclcpp geometry_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(kinematics_node rclcpp geometry_msgs tf2 tf2_geometry_msgs)

#install
install(TARGETS
  state_reporter_node
  motion_planner_node
  kinematics_node
  DESTINATION lib/${PROJECT_NAME})

#Install launch files
install(DIRECTORY launch/
 DESTINATION share/${PROJECT_NAME}/)

ament_package()