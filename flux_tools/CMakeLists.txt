cmake_minimum_required(VERSION 3.16)
project(flux_tools VERSION 0.0.0 LANGUAGES CXX)

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
find_package(flux_cpp REQUIRED)
find_package(flux_gen REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# The display reads the frame through the sensor_msgs/Image adapter, generated here from the
# installed .msg so the plugin and any publisher built the same way share one fingerprint.
set(_adapters "${CMAKE_CURRENT_BINARY_DIR}/flux_adapters")
set(_image_msg "${sensor_msgs_DIR}/../msg/Image.msg")
file(GLOB _flux_gen_sources CONFIGURE_DEPENDS "${FLUX_GEN_PYTHON_DIR}/flux_gen/*.py")
add_custom_command(
  OUTPUT "${_adapters}/sensor_msgs/flux/image.hpp"
  COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${FLUX_GEN_PYTHON_DIR}:$ENV{PYTHONPATH}"
          ${Python3_EXECUTABLE} -m flux_gen --pkg sensor_msgs
          --msg "${_image_msg}" --out-cpp "${_adapters}"
  DEPENDS "${_image_msg}" ${_flux_gen_sources}
  COMMENT "flux_gen: sensor_msgs/Image adapter"
  VERBATIM)

add_library(flux_tools_rviz SHARED
  src/image_display.cpp
  include/flux_tools/rviz/image_display.hpp
  "${_adapters}/sensor_msgs/flux/image.hpp")
target_include_directories(flux_tools_rviz
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE "${_adapters}")
target_link_libraries(flux_tools_rviz
  flux_cpp::flux_cpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_ogre_vendor::OgreMain
  Qt5::Widgets)
ament_target_dependencies(flux_tools_rviz pluginlib rclcpp)
target_compile_options(flux_tools_rviz PRIVATE -Wall -Wextra -Wpedantic)
# Ogre and Qt headers are not -Wpedantic clean; the warnings they emit are not ours to fix.
target_compile_options(flux_tools_rviz PRIVATE -Wno-pedantic)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS flux_tools_rviz
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
