cmake_minimum_required(VERSION 3.8)
project(mecanum-car)

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
find_package(interfaces REQUIRED)

add_executable(hello_node src/hello_node.cpp)

add_executable(guidance src/guidance.cpp)
ament_target_dependencies(guidance rclcpp interfaces)

add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop rclcpp interfaces)

install(TARGETS
  hello_node
  guidance
  teleop
  DESTINATION lib/${PROJECT_NAME})

ament_package()
