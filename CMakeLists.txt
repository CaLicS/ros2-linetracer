cmake_minimum_required(VERSION 3.16)
project(line_tracer)

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
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(/usr/local/include/dynamixel_sdk)
#build
add_executable(jetson src/jetson.cpp src/jetson_main.cpp src/dxl.cpp /home/jetson/ros2_ws/src/line_tracer/src/8_lt_cw_100rpm_in.mp4)
ament_target_dependencies(jetson rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(jetson dxl_x64_cpp)

add_executable(wsl src/wsl.cpp src/wsl_main.cpp src/dxl.cpp)
ament_target_dependencies(wsl rclcpp sensor_msgs std_msgs OpenCV)
target_link_libraries(wsl dxl_x64_cpp)

# Install
install(TARGETS
  jetson
  wsl
  DESTINATION lib/${PROJECT_NAME})

ament_package()