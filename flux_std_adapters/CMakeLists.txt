cmake_minimum_required(VERSION 3.16)
project(flux_std_adapters)

find_package(ament_cmake REQUIRED)
find_package(flux_gen REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# flux_generate_adapters() names the adapter after the calling package, so it cannot emit one for
# a message another package owns. These are generated from the installed .msg instead: one copy
# per install, so every consumer shares one C++ definition and one Python module.
set(_flux_types
  sensor_msgs/CameraInfo
  sensor_msgs/CompressedImage
  sensor_msgs/Image
  sensor_msgs/Imu
  sensor_msgs/LaserScan
  sensor_msgs/PointCloud2
  stereo_msgs/DisparityImage
  nav_msgs/OccupancyGrid
  nav_msgs/Odometry)

set(_flux_root "${CMAKE_CURRENT_BINARY_DIR}/flux_adapters")
set(_flux_outputs "")

foreach(_flux_type IN LISTS _flux_types)
  string(REPLACE "/" ";" _flux_parts "${_flux_type}")
  list(GET _flux_parts 0 _flux_pkg)
  list(GET _flux_parts 1 _flux_name)
  find_package(${_flux_pkg} REQUIRED)
  _flux_generate(${_flux_pkg} "${${_flux_pkg}_DIR}/../msg/${_flux_name}.msg" ON "${_flux_root}"
    _flux_files)
  list(APPEND _flux_outputs ${_flux_files})
endforeach()

add_custom_target(${PROJECT_NAME} ALL DEPENDS ${_flux_outputs})

install(DIRECTORY "${_flux_root}/include/" DESTINATION "include/${PROJECT_NAME}")
# No __init__.py: <pkg>_flux stays a namespace package, so a module generated elsewhere for a type
# not listed here can sit beside these on the path.
install(DIRECTORY "${_flux_root}/python/"
  DESTINATION "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

# The headers include flux_cpp and each message package's rosidl headers.
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_dependencies(flux_cpp nav_msgs sensor_msgs stereo_msgs)
ament_package()
