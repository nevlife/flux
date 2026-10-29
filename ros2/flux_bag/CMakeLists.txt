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
find_package(flux_std_adapters REQUIRED)
find_package(JPEG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)

# Humble's rosbag2 (0.15) keeps one time per message, has no TopicFilter, and its Player::play()
# blocks without the playback hooks; the code reads which by this minor version.
add_compile_definitions(FLUX_ROSBAG2_MINOR=${rosbag2_transport_VERSION_MINOR})

add_executable(record src/record.cpp)
target_include_directories(record PRIVATE include ${flux_bridge_INCLUDE_DIRS}
  ${flux_std_adapters_INCLUDE_DIRS})
target_link_libraries(record
  flux_cpp::flux_cpp
  JPEG::JPEG
  rclcpp::rclcpp
  rcutils::rcutils
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rosbag2_transport::rosbag2_transport)
target_compile_options(record PRIVATE -Wall -Wextra -Wpedantic)

add_executable(play src/play.cpp)
target_include_directories(play PRIVATE include ${flux_bridge_INCLUDE_DIRS}
  ${flux_std_adapters_INCLUDE_DIRS})
target_link_libraries(play
  flux_cpp::flux_cpp
  JPEG::JPEG
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rosbag2_transport::rosbag2_transport)
target_compile_options(play PRIVATE -Wall -Wextra -Wpedantic)

add_executable(convert src/convert.cpp)
target_include_directories(convert PRIVATE include ${flux_std_adapters_INCLUDE_DIRS})
target_link_libraries(convert
  flux_cpp::flux_cpp
  JPEG::JPEG
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage)
target_compile_options(convert PRIVATE -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(flux_msgs REQUIRED)
  ament_add_gtest(test_flux_frame_cdr test/test_flux_frame_cdr.cpp)
  target_include_directories(test_flux_frame_cdr PRIVATE include)
  target_link_libraries(test_flux_frame_cdr flux_cpp::flux_cpp rclcpp::rclcpp
    ${flux_msgs_TARGETS})

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_verbs test/test_verbs.py)
  ament_add_pytest_test(test_record_play test/test_record_play.py TIMEOUT 120)
  ament_add_pytest_test(test_play_control test/test_play_control.py TIMEOUT 120)
  ament_add_pytest_test(test_record_live test/test_record_live.py TIMEOUT 120)

  foreach(poll 0 -1)
    add_test(NAME record_refuses_poll_${poll} COMMAND record --poll ${poll})
    set_tests_properties(record_refuses_poll_${poll} PROPERTIES
      PASS_REGULAR_EXPRESSION "argument --poll: Value must be positive" TIMEOUT 10)
  endforeach()
  foreach(poll abc 1s)
    add_test(NAME record_refuses_poll_${poll} COMMAND record --poll ${poll})
    set_tests_properties(record_refuses_poll_${poll} PROPERTIES
      PASS_REGULAR_EXPRESSION "argument --poll: Expects a floating point number" TIMEOUT 10)
  endforeach()
  foreach(poll nan inf)
    add_test(NAME record_refuses_poll_${poll} COMMAND record --poll ${poll})
    set_tests_properties(record_refuses_poll_${poll} PROPERTIES
      PASS_REGULAR_EXPRESSION "argument --poll: Value must be finite" TIMEOUT 10)
  endforeach()
  foreach(rate 2abc nan inf 0)
    add_test(NAME play_refuses_rate_${rate} COMMAND play some_bag -r ${rate})
    set_tests_properties(play_refuses_rate_${rate} PROPERTIES
      PASS_REGULAR_EXPRESSION "argument -r: " TIMEOUT 10)
  endforeach()
endif()

install(TARGETS record play convert DESTINATION lib/${PROJECT_NAME})

# The ros2 bag verbs (setup.cfg) are how a person runs the three programs above.
ament_python_install_package(${PROJECT_NAME})

ament_package()
