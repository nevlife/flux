cmake_minimum_required(VERSION 3.16)
project(flux_wrapper VERSION 0.0.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# The build environment decides, not whether flux happens to be found: package.xml declares the
# flux dependency under the same condition, so colcon builds flux first exactly when it is used.
if("$ENV{ENABLE_FLUX}" STREQUAL "1")
  set(FLUX_WRAPPER_WITH_FLUX 1)
else()
  set(FLUX_WRAPPER_WITH_FLUX 0)
endif()

add_library(flux_wrapper INTERFACE)
target_include_directories(flux_wrapper INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(flux_wrapper INTERFACE
  rclcpp::rclcpp ${builtin_interfaces_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS})
set(flux_dependencies)
if(FLUX_WRAPPER_WITH_FLUX)
  find_package(flux_cpp REQUIRED)
  find_package(flux_std_adapters REQUIRED)
  target_link_libraries(flux_wrapper INTERFACE flux_cpp::flux_cpp)
  target_include_directories(flux_wrapper INTERFACE ${flux_std_adapters_INCLUDE_DIRS})
  target_compile_definitions(flux_wrapper INTERFACE USE_FLUX_ENABLED)
  set(flux_dependencies flux_cpp flux_std_adapters)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest_executable(test_flux_wrapper test/test_flux_wrapper.cpp)
  target_link_libraries(test_flux_wrapper flux_wrapper)
  target_compile_options(test_flux_wrapper PRIVATE -Wall -Wextra -Wpedantic -Werror)
  # Each run switch the build can take; a build without flux takes only the ROS path.
  set(runs 0)
  if(FLUX_WRAPPER_WITH_FLUX)
    list(APPEND runs 1)
  endif()
  foreach(run ${runs})
    ament_add_gtest_test(test_flux_wrapper TEST_NAME test_flux_wrapper_run${run}
      ENV ENABLE_FLUX=${run} FLUX_WRAPPER_EXPECT_FLUX=${run} TIMEOUT 60)
    ament_add_pytest_test(test_flux_wrapper_py_run${run} test/test_flux_wrapper.py
      ENV ENABLE_FLUX=${run} FLUX_WRAPPER_EXPECT_FLUX=${run} TIMEOUT 60)
  endforeach()
  if(FLUX_WRAPPER_WITH_FLUX)
    ament_add_gtest_test(test_flux_wrapper TEST_NAME test_flux_wrapper_unset
      ENV FLUX_WRAPPER_EXPECT_FLUX=0 TIMEOUT 60)
  endif()
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS flux_wrapper EXPORT export_flux_wrapper)
ament_export_targets(export_flux_wrapper)
ament_export_dependencies(builtin_interfaces rclcpp sensor_msgs std_msgs ${flux_dependencies})

ament_python_install_package(${PROJECT_NAME})

ament_package(CONFIG_EXTRAS cmake/flux_wrapper-extras.cmake.in)
