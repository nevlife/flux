cmake_minimum_required(VERSION 3.16)
project(flux2_tools VERSION 0.0.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(flux2_cpp REQUIRED)
find_package(flux2_std_adapters REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
# Qt comes with rviz_common, which finds the Qt it was built with (6 from Lyrical on, 5 on
# Jazzy). Looking for Qt6 here would find one rviz on Jazzy does not use.

add_library(flux2_tools_rviz SHARED
  src/channel_display.cpp
  src/depth_cloud_display.cpp
  src/image_display.cpp
  src/point_cloud2_display.cpp
  include/flux2_tools/cloud_from_frame.hpp
  include/flux2_tools/depth_projection.hpp
  include/flux2_tools/rviz/channel_display.hpp
  include/flux2_tools/rviz/depth_cloud_display.hpp
  include/flux2_tools/rviz/image_display.hpp
  include/flux2_tools/rviz/point_cloud2_display.hpp
  include/flux2_tools/rviz/update_dt.hpp
  include/flux2_tools/whole_pixel_rows.hpp)
target_include_directories(flux2_tools_rviz
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${flux2_std_adapters_INCLUDE_DIRS})
target_link_libraries(flux2_tools_rviz
  flux2_cpp::flux2_cpp
  image_transport::image_transport
  pluginlib::pluginlib
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_default_plugins::rviz_default_plugins
  rviz_rendering::rviz_rendering
  rviz_ogre_vendor::OgreMain)
target_compile_options(flux2_tools_rviz PRIVATE -Wall -Wextra)
if(rviz_common_VERSION VERSION_GREATER_EQUAL 15)
  target_compile_definitions(flux2_tools_rviz PRIVATE FLUX2_TOOLS_CHRONO_UPDATE)
endif()

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_projection test/test_depth_projection.cpp)
  target_include_directories(test_depth_projection PRIVATE include)
  target_link_libraries(test_depth_projection ${sensor_msgs_TARGETS})
  ament_add_gtest(test_whole_pixel_rows test/test_whole_pixel_rows.cpp)
  target_include_directories(test_whole_pixel_rows PRIVATE include)
  ament_add_gtest(test_cloud_from_frame test/test_cloud_from_frame.cpp)
  target_include_directories(test_cloud_from_frame PRIVATE include ${flux2_std_adapters_INCLUDE_DIRS})
  target_link_libraries(test_cloud_from_frame flux2_cpp::flux2_cpp ${sensor_msgs_TARGETS})
endif()

install(TARGETS flux2_tools_rviz
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
