cmake_minimum_required(VERSION 3.8)
project(turtlesim_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(moving_turtle src/moving_turtle.cpp)
ament_target_dependencies(moving_turtle rclcpp geometry_msgs)

add_executable(color_changer src/color_changer.cpp)
ament_target_dependencies(color_changer rclcpp turtlesim custom_interfaces)

add_executable(hmi src/hmi.cpp)
ament_target_dependencies(hmi rclcpp turtlesim custom_interfaces visualization_msgs)

install(TARGETS
  moving_turtle
  color_changer
  hmi
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
