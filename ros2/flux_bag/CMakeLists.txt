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
find_package(ament_cmake_python REQUIRED)
find_package(flux_bridge REQUIRED)
find_package(flux_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# FluxFrame as record_flux stores it, for other recorders too.
add_library(flux_bag_frames SHARED src/frame_encoder.cpp)
target_include_directories(flux_bag_frames PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(flux_bag_frames PUBLIC flux_cpp::flux_cpp)
target_compile_options(flux_bag_frames PRIVATE -Wall -Wextra -Wpedantic)

add_executable(record src/record.cpp)
target_include_directories(record PRIVATE include ${flux_bridge_INCLUDE_DIRS})
target_link_libraries(record
  flux_bag_frames
  flux_cpp::flux_cpp
  rclcpp::rclcpp
  rcutils::rcutils
  rosbag2_compression::rosbag2_compression
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rosbag2_transport::rosbag2_transport
  yaml-cpp)
target_compile_options(record PRIVATE -Wall -Wextra -Wpedantic)

add_executable(play src/play.cpp)
target_include_directories(play PRIVATE include)
target_link_libraries(play
  flux_cpp::flux_cpp
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rosbag2_transport::rosbag2_transport
  yaml-cpp)
target_compile_options(play PRIVATE -Wall -Wextra -Wpedantic)

add_executable(convert src/convert.cpp)
target_include_directories(convert PRIVATE include)
target_link_libraries(convert
  flux_cpp::flux_cpp
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rosbag2_transport::rosbag2_transport)
target_compile_options(convert PRIVATE -Wall -Wextra -Wpedantic)

# libFuzzer targets: -DFLUX_FUZZ=ON with clang (-DCMAKE_CXX_COMPILER=clang++).
option(FLUX_FUZZ "Build the libFuzzer targets" OFF)
if(FLUX_FUZZ)
  add_executable(fuzz_frame test/fuzz_frame.cpp)
  target_compile_definitions(fuzz_frame PRIVATE FLUX_LIBFUZZER)
  target_compile_options(fuzz_frame PRIVATE -fsanitize=fuzzer,address,undefined)
  target_link_options(fuzz_frame PRIVATE -fsanitize=fuzzer,address,undefined)
  target_include_directories(fuzz_frame PRIVATE include)
  target_link_libraries(fuzz_frame flux_cpp::flux_cpp)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(flux_msgs REQUIRED)
  ament_add_gtest(test_flux_frame_cdr test/test_flux_frame_cdr.cpp)
  target_include_directories(test_flux_frame_cdr PRIVATE include)
  target_link_libraries(test_flux_frame_cdr flux_cpp::flux_cpp rclcpp::rclcpp
    ${flux_msgs_TARGETS})

  ament_add_gtest(test_fuzz_frame test/fuzz_frame.cpp)
  target_include_directories(test_fuzz_frame PRIVATE include)
  target_link_libraries(test_fuzz_frame flux_cpp::flux_cpp)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_verbs test/test_verbs.py)
  ament_add_pytest_test(test_record_play test/test_record_play.py TIMEOUT 120)
  ament_add_pytest_test(test_play_control test/test_play_control.py TIMEOUT 120)
  ament_add_pytest_test(test_record_live test/test_record_live.py TIMEOUT 120)

  # The verbs are how the programs get their options; by hand they only say so.
  foreach(program record play)
    add_test(NAME ${program}_takes_an_options_file COMMAND ${program})
    set_tests_properties(${program}_takes_an_options_file PROPERTIES
      PASS_REGULAR_EXPRESSION "expects the options file" TIMEOUT 10)
  endforeach()
endif()

install(TARGETS record play convert DESTINATION lib/${PROJECT_NAME})
install(TARGETS flux_bag_frames EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(FILES include/flux_bag/flux_frame_cdr.hpp include/flux_bag/frame_encoder.hpp
  include/flux_bag/raw_meta.hpp DESTINATION include/${PROJECT_NAME}/flux_bag)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(flux_cpp)

# The ros2 bag verbs (setup.cfg) are how a person runs the three programs above.
ament_python_install_package(${PROJECT_NAME})

ament_package()
