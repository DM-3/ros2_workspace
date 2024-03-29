cmake_minimum_required(VERSION 3.8)
project(timing_tubaf)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Cpp
add_executable(my_publisher src/my_publisher.cpp)
ament_target_dependencies(my_publisher rclcpp std_msgs)

install(TARGETS 
  my_publisher
  DESTINATION lib/${PROJECT_NAME})

# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/my_subscriber.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
