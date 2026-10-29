cmake_minimum_required(VERSION 3.16)
project(flux_bridge VERSION 0.0.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(flux_cpp REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(bridge src/bridge.cpp)
target_include_directories(bridge PRIVATE include)
target_link_libraries(bridge flux_cpp::flux_cpp rclcpp::rclcpp)
target_compile_options(bridge PRIVATE -Wall -Wextra -Wpedantic)

# libFuzzer targets: -DFLUX_FUZZ=ON with clang (-DCMAKE_CXX_COMPILER=clang++).
option(FLUX_FUZZ "Build the libFuzzer targets" OFF)
if(FLUX_FUZZ)
  find_package(flux_std_adapters REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(stereo_msgs REQUIRED)
  add_executable(fuzz_views test/fuzz_views.cpp)
  target_compile_definitions(fuzz_views PRIVATE FLUX_LIBFUZZER)
  target_compile_options(fuzz_views PRIVATE -fsanitize=fuzzer,address,undefined)
  target_link_options(fuzz_views PRIVATE -fsanitize=fuzzer,address,undefined)
  target_include_directories(fuzz_views PRIVATE ${flux_std_adapters_INCLUDE_DIRS})
  target_link_libraries(fuzz_views flux_cpp::flux_cpp
    ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${stereo_msgs_TARGETS})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(flux_std_adapters REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(stereo_msgs REQUIRED)
  ament_add_gtest(test_cdr test/test_cdr.cpp)
  target_include_directories(test_cdr PRIVATE ${flux_std_adapters_INCLUDE_DIRS})
  target_link_libraries(test_cdr flux_cpp::flux_cpp rclcpp::rclcpp
    ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${stereo_msgs_TARGETS})
  target_compile_options(test_cdr PRIVATE -Wall -Wextra -Werror)
  ament_add_gtest(test_fuzz_views test/fuzz_views.cpp)
  target_include_directories(test_fuzz_views PRIVATE ${flux_std_adapters_INCLUDE_DIRS})
  target_link_libraries(test_fuzz_views flux_cpp::flux_cpp
    ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${stereo_msgs_TARGETS})
  target_compile_options(test_fuzz_views PRIVATE -Wall -Wextra -Werror)
  ament_add_gtest(test_mirror test/test_mirror.cpp TIMEOUT 60)
  target_include_directories(test_mirror PRIVATE ${flux_std_adapters_INCLUDE_DIRS})
  target_link_libraries(test_mirror flux_cpp::flux_cpp rclcpp::rclcpp ${sensor_msgs_TARGETS})
  target_compile_options(test_mirror PRIVATE -Wall -Wextra -Werror)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_flux_bridge test/test_flux_bridge.py TIMEOUT 120)

  foreach(poll 0 -1)
    add_test(NAME bridge_refuses_poll_${poll} COMMAND bridge --poll ${poll})
    set_tests_properties(bridge_refuses_poll_${poll} PROPERTIES
      PASS_REGULAR_EXPRESSION "argument --poll: Value must be positive" TIMEOUT 10)
  endforeach()
  foreach(poll abc 1s 0x1p0)
    add_test(NAME bridge_refuses_poll_${poll} COMMAND bridge --poll ${poll})
    set_tests_properties(bridge_refuses_poll_${poll} PROPERTIES
      PASS_REGULAR_EXPRESSION "argument --poll: Expects a floating point number" TIMEOUT 10)
  endforeach()
  foreach(poll nan inf)
    add_test(NAME bridge_refuses_poll_${poll} COMMAND bridge --poll ${poll})
    set_tests_properties(bridge_refuses_poll_${poll} PROPERTIES
      PASS_REGULAR_EXPRESSION "argument --poll: Value must be finite" TIMEOUT 10)
  endforeach()
  # Refused for what follows it, so the --poll=SEC before it was taken.
  add_test(NAME bridge_takes_poll_equals COMMAND bridge --poll=0.5 --bogus)
  set_tests_properties(bridge_takes_poll_equals PROPERTIES
    PASS_REGULAR_EXPRESSION "unknown argument --bogus" TIMEOUT 10)
  add_test(NAME bridge_help COMMAND bridge -h)
  set_tests_properties(bridge_help PROPERTIES FAIL_REGULAR_EXPRESSION "unknown argument" TIMEOUT 10)
endif()

install(TARGETS bridge DESTINATION lib/${PROJECT_NAME})
# tools.hpp: flux_bag's recorder parses --poll and checks channel keys the same way.
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})

# The ros2 topic verbs (setup.cfg) reach the bridge this program runs.
ament_python_install_package(${PROJECT_NAME})

ament_package()
