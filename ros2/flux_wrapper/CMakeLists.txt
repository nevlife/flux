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
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
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
# The introspection type support copies a message but its data (detail/copy_but_data.hpp).
target_link_libraries(flux_wrapper INTERFACE
  rclcpp::rclcpp rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
  ${builtin_interfaces_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS})
set(flux_dependencies)
if(FLUX_WRAPPER_WITH_FLUX)
  find_package(flux_cpp REQUIRED)
  find_package(flux_std_adapters REQUIRED)
  target_link_libraries(flux_wrapper INTERFACE flux_cpp::flux_cpp)
  target_include_directories(flux_wrapper INTERFACE ${flux_std_adapters_INCLUDE_DIRS})
  target_compile_definitions(flux_wrapper INTERFACE USE_FLUX_ENABLED)
  set(flux_dependencies flux_cpp flux_std_adapters)
  # Python reads the build's choice here, so it takes the flux path exactly when C++ does.
  ament_index_register_resource(flux_wrapper_with_flux CONTENT "1")
endif()

# Device::Cuda works only in a build that found the CUDA toolkit; elsewhere it falls back to Host.
find_package(CUDAToolkit QUIET)
if(CUDAToolkit_FOUND)
  set(FLUX_WRAPPER_WITH_CUDA 1)
  target_link_libraries(flux_wrapper INTERFACE CUDA::cudart)
  target_compile_definitions(flux_wrapper INTERFACE FLUX_WRAPPER_CUDA)
else()
  set(FLUX_WRAPPER_WITH_CUDA 0)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(flux_wrapper_test_msgs REQUIRED)
  ament_add_gtest_executable(test_flux_wrapper test/test_flux_wrapper.cpp)
  target_link_libraries(test_flux_wrapper flux_wrapper ${flux_wrapper_test_msgs_TARGETS})
  target_compile_options(test_flux_wrapper PRIVATE -Wall -Wextra -Wpedantic -Werror)
  # Each run switch the build can take; a build without flux takes only the ROS path.
  set(runs 0)
  if(FLUX_WRAPPER_WITH_FLUX)
    list(APPEND runs 1)
  endif()
  foreach(run ${runs})
    ament_add_gtest_test(test_flux_wrapper TEST_NAME test_flux_wrapper_run${run}
      ENV ENABLE_FLUX=${run} FLUX_WRAPPER_EXPECT_FLUX=${run}
      FLUX_WRAPPER_PY_TEST=${CMAKE_CURRENT_SOURCE_DIR}/test/test_flux_wrapper.py TIMEOUT 60)
    ament_add_pytest_test(test_flux_wrapper_py_run${run} test/test_flux_wrapper.py
      ENV ENABLE_FLUX=${run} FLUX_WRAPPER_EXPECT_FLUX=${run} TIMEOUT 60)
  endforeach()
  if(FLUX_WRAPPER_WITH_FLUX)
    # env -u: the run must not inherit ENABLE_FLUX from the shell that started the tests.
    set(result "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_flux_wrapper_unset.gtest.xml")
    ament_add_test(test_flux_wrapper_unset
      COMMAND env -u ENABLE_FLUX $<TARGET_FILE:test_flux_wrapper> --gtest_output=xml:${result}
      OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cmake_gtest/test_flux_wrapper_unset.txt"
      RESULT_FILE "${result}"
      ENV FLUX_WRAPPER_EXPECT_FLUX=0 TIMEOUT 60)
    # A type without an adapter stops a build with flux at the static_assert that says what to
    # include. The test builds the target and passes on that message, not on the exit code.
    add_executable(test_no_adapter EXCLUDE_FROM_ALL test/no_adapter.cpp)
    target_link_libraries(test_no_adapter flux_wrapper)
    add_test(NAME test_no_adapter
      COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target test_no_adapter)
    set_tests_properties(test_no_adapter PROPERTIES
      PASS_REGULAR_EXPRESSION "no flux adapter for this message type" TIMEOUT 120)
  endif()
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS flux_wrapper EXPORT export_flux_wrapper)
ament_export_targets(export_flux_wrapper)
ament_export_dependencies(
  builtin_interfaces rclcpp rosidl_typesupport_introspection_cpp sensor_msgs std_msgs
  ${flux_dependencies})

ament_python_install_package(${PROJECT_NAME})

ament_package(CONFIG_EXTRAS cmake/flux_wrapper-extras.cmake.in)
