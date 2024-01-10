cmake_minimum_required(VERSION 3.8)
project(vision_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(image_converter src/ImageConverter.cpp)
ament_target_dependencies(image_converter rclcpp std_msgs sensor_msgs cv_bridge)

install(TARGETS
image_converter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
