cmake_minimum_required(VERSION 3.16)
project(flux_example_executor)

# colcon passes no build type; without this the package compiles at -O0 (flux_core/CMakeLists.txt).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(flux_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(flux_example_msgs REQUIRED)

foreach(node merged_sub partitioned_sub split_sub service_sub multi_sub)
  add_executable(${node} src/${node}.cpp)
  target_link_libraries(${node} flux_cpp::flux_cpp)
  ament_target_dependencies(${node} rclcpp sensor_msgs std_srvs flux_example_msgs)
  target_compile_options(${node} PRIVATE -Wall -Wextra -Wpedantic)
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(PROGRAMS
  ${PROJECT_NAME}/merged_sub.py
  ${PROJECT_NAME}/partitioned_sub.py
  ${PROJECT_NAME}/split_sub.py
  ${PROJECT_NAME}/service_sub.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
