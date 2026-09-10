cmake_minimum_required(VERSION 3.16)
project(flux_example_rt)

# colcon passes no build type; without this the package compiles at -O0 (flux_core/CMakeLists.txt).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(flux_cpp REQUIRED)
find_package(flux_example_msgs REQUIRED)

add_executable(rt_preflight src/rt_preflight.cpp)
target_link_libraries(rt_preflight flux_cpp::flux_cpp)
target_compile_options(rt_preflight PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS rt_preflight DESTINATION lib/${PROJECT_NAME})

add_executable(rt_chain src/rt_chain.cpp)
target_link_libraries(rt_chain flux_cpp::flux_cpp)
ament_target_dependencies(rt_chain rclcpp flux_example_msgs)
target_compile_options(rt_chain PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS rt_chain DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
