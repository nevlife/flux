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
file(GLOB _flux_gen_sources CONFIGURE_DEPENDS "${FLUX_GEN_PYTHON_DIR}/flux_gen/*.py")
set(_flux_outputs "")

foreach(_flux_type IN LISTS _flux_types)
  string(REPLACE "/" ";" _flux_parts "${_flux_type}")
  list(GET _flux_parts 0 _flux_pkg)
  list(GET _flux_parts 1 _flux_name)
  find_package(${_flux_pkg} REQUIRED)
  set(_flux_msg "${${_flux_pkg}_DIR}/../msg/${_flux_name}.msg")

  # Same snake_case rule as flux_generate_adapters().
  string(REGEX REPLACE "(.)([A-Z][a-z]+)" "\\1_\\2" _flux_snake "${_flux_name}")
  string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" _flux_snake "${_flux_snake}")
  string(TOLOWER "${_flux_snake}" _flux_snake)

  set(_flux_out
    "${_flux_root}/include/${_flux_pkg}/flux/${_flux_snake}.hpp"
    "${_flux_root}/include/${_flux_pkg}/flux/${_flux_snake}_ros.hpp"
    "${_flux_root}/python/${_flux_pkg}_flux/${_flux_snake}.py"
    "${_flux_root}/python/${_flux_pkg}_flux/${_flux_snake}_ros.py")
  set(_flux_dep "${_flux_root}/${_flux_pkg}_${_flux_snake}.d")
  add_custom_command(
    OUTPUT ${_flux_out}
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${FLUX_GEN_PYTHON_DIR}:$ENV{PYTHONPATH}"
            ${Python3_EXECUTABLE} -m flux_gen --pkg ${_flux_pkg}
            --msg "${_flux_msg}"
            --out-cpp "${_flux_root}/include"
            --out-py "${_flux_root}/python"
            --depfile "${_flux_dep}"
    DEPENDS "${_flux_msg}" ${_flux_gen_sources}
    DEPFILE "${_flux_dep}"
    COMMENT "flux_gen: ${_flux_type}"
    VERBATIM)
  list(APPEND _flux_outputs ${_flux_out})
endforeach()

add_custom_target(${PROJECT_NAME} ALL DEPENDS ${_flux_outputs})

install(DIRECTORY "${_flux_root}/include/" DESTINATION "include/${PROJECT_NAME}")
# No __init__.py: <pkg>_flux stays a namespace package, so a module generated elsewhere for a type
# not listed here can sit beside these on the path.
install(DIRECTORY "${_flux_root}/python/"
  DESTINATION "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

# The headers include flux_core and each message package's rosidl headers.
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_dependencies(flux_core nav_msgs sensor_msgs stereo_msgs)
ament_package()
