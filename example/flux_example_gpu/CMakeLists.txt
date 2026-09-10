cmake_minimum_required(VERSION 3.16)
project(flux_example_gpu)

# colcon passes no build type; without this the package compiles at -O0 (flux_core/CMakeLists.txt).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(flux_cpp REQUIRED)

foreach(node image_pub_gpu image_sub_gpu)
  add_executable(${node} src/${node}.cpp)
  target_link_libraries(${node} flux_cpp::flux_cpp)
  ament_target_dependencies(${node} rclcpp)
  target_compile_options(${node} PRIVATE -Wall -Wextra -Wpedantic)
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(PROGRAMS
  ${PROJECT_NAME}/image_pub_gpu.py
  ${PROJECT_NAME}/image_sub_gpu.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
