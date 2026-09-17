cmake_minimum_required(VERSION 3.16)
project(flux_bag VERSION 0.0.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(flux_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)

add_executable(record src/record.cpp)
target_include_directories(record PRIVATE include)
target_link_libraries(record
  flux_cpp::flux_cpp
  rclcpp::rclcpp
  rcutils::rcutils
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rosbag2_transport::rosbag2_transport)
target_compile_options(record PRIVATE -Wall -Wextra -Wpedantic)

add_executable(play src/play.cpp)
target_include_directories(play PRIVATE include)
target_link_libraries(play
  flux_cpp::flux_cpp
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rosbag2_transport::rosbag2_transport)
target_compile_options(play PRIVATE -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(flux_msgs REQUIRED)
  ament_add_gtest(test_flux_frame_cdr test/test_flux_frame_cdr.cpp)
  target_include_directories(test_flux_frame_cdr PRIVATE include)
  target_link_libraries(test_flux_frame_cdr flux_cpp::flux_cpp rclcpp::rclcpp
    ${flux_msgs_TARGETS})
endif()

install(TARGETS record play DESTINATION lib/${PROJECT_NAME})

ament_package()
