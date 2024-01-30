cmake_minimum_required(VERSION 3.8)
project(camera_line)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(main 
  "include/camera_line/follower.hpp"
  "src/follower.cpp"
  "src/main.cpp")

target_include_directories(main PRIVATE "include")
ament_target_dependencies(main
  rclcpp
  sensor_msgs
  geometry_msgs
  OpenCV
  cv_bridge)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  "launch.yaml"
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
