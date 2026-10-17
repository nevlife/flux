cmake_minimum_required(VERSION 3.16)
project(flux_cpp VERSION 0.1.0 LANGUAGES CXX)

# flux for rclcpp nodes, over flux's C ABI.

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The Rust core stays uninstrumented: thread finds races in this layer, not ones the core's atomics
# order. A separate build base keeps it apart from the normal build:
# colcon build --build-base build_tsan --install-base install_tsan --cmake-args -DFLUX_SANITIZER=thread
set(FLUX_SANITIZER "" CACHE STRING "Build with -fsanitize=<value> (thread, address, undefined)")
if(FLUX_SANITIZER)
  add_compile_options(-fsanitize=${FLUX_SANITIZER} -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=${FLUX_SANITIZER})
endif()

find_package(ament_cmake REQUIRED)
find_package(flux_ffi REQUIRED)
find_package(rclcpp REQUIRED)
# For flux/message_filters/subscriber.hpp, a public header.
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)

add_library(flux_cpp
  src/publisher.cpp
  src/subscription.cpp
  src/executor.cpp
  src/partitioned_executor.cpp)
add_library(flux::cpp ALIAS flux_cpp)
set_target_properties(flux_cpp PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(flux_cpp PUBLIC cxx_std_17)
target_compile_options(flux_cpp PRIVATE -Wall -Wextra -Werror)
target_include_directories(flux_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(flux_cpp PUBLIC
  flux::ffi rclcpp::rclcpp message_filters::message_filters ${std_msgs_TARGETS})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS flux_cpp EXPORT flux_cppTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
ament_export_targets(flux_cppTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(flux_ffi rclcpp message_filters std_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(sensor_msgs REQUIRED)
  foreach(t test_channel test_executor test_ros test_partitioned test_message_filters test_gpu)
    ament_add_gtest(${t} test/${t}.cpp TIMEOUT 120)
    target_link_libraries(${t} flux_cpp ${std_msgs_TARGETS} ${sensor_msgs_TARGETS})
    target_compile_options(${t} PRIVATE -Wall -Wextra -Werror)
  endforeach()
  target_compile_definitions(test_gpu PRIVATE
    FLUX_KERNEL_LOG="${CMAKE_CURRENT_SOURCE_DIR}/../scripts/kernel_log.sh")
  # Again on the parker threads a kernel below 5.16 waits with. ament_add_gtest_test takes no
  # TEST_NAME on Humble.
  foreach(t test_executor test_ros test_partitioned)
    set(_result ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${t}_parked.gtest.xml)
    ament_add_test(${t}_parked
      COMMAND $<TARGET_FILE:${t}> --gtest_output=xml:${_result}
      RESULT_FILE ${_result}
      TIMEOUT 120
      ENV FLUX_DISABLE_FUTEX_WAITV=1)
  endforeach()
  # Deadline arithmetic in the headers must not overflow; this makes an overflow abort the test.
  target_compile_options(test_channel PRIVATE
    -fsanitize=signed-integer-overflow -fno-sanitize-recover=signed-integer-overflow)
  target_link_options(test_channel PRIVATE -fsanitize=signed-integer-overflow)
  # The message_filters test reads a generated adapter, so generator, wire runtime and filter
  # are checked together.
  find_package(flux_gen REQUIRED)
  set(_adapters ${CMAKE_CURRENT_BINARY_DIR}/test_adapters)
  add_custom_command(
    OUTPUT ${_adapters}/flux_test/flux/stamped.hpp
    COMMAND "${FLUX_GEN_EXECUTABLE}" --pkg flux_test
            --msg ${CMAKE_CURRENT_SOURCE_DIR}/test/msg/Stamped.msg --out-cpp ${_adapters}
    DEPENDS test/msg/Stamped.msg
    VERBATIM)
  add_custom_target(flux_cpp_test_adapters DEPENDS ${_adapters}/flux_test/flux/stamped.hpp)
  add_dependencies(test_message_filters flux_cpp_test_adapters)
  target_include_directories(test_message_filters PRIVATE ${_adapters})
endif()

ament_package()
