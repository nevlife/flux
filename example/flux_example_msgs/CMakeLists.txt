cmake_minimum_required(VERSION 3.16)
project(flux_example_msgs)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(flux_gen REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# flux_generate_adapters() takes package-relative paths, and these .msg files belong to
# sensor_msgs. Generating from the installed copy rather than a vendored one is what keeps the
# flux frame and the ROS message on one schema.
set(_adapters "${CMAKE_CURRENT_BINARY_DIR}/flux_adapters")
set(_msg_dir "${sensor_msgs_DIR}/../msg")
# The emitters are an input: renaming what the generator produces has to fail the build, and it
# only does if an edit to them regenerates.
file(GLOB _flux_gen_sources CONFIGURE_DEPENDS "${FLUX_GEN_PYTHON_DIR}/flux_gen/*.py")

set(_generated "")
foreach(pair "Image:image" "PointCloud2:point_cloud2")
  string(REPLACE ":" ";" pair "${pair}")
  list(GET pair 0 _name)
  list(GET pair 1 _snake)
  add_custom_command(
    OUTPUT "${_adapters}/include/sensor_msgs/flux/${_snake}.hpp"
           "${_adapters}/python/sensor_msgs_flux/${_snake}.py"
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${FLUX_GEN_PYTHON_DIR}:$ENV{PYTHONPATH}"
            ${Python3_EXECUTABLE} -m flux_gen --pkg sensor_msgs
            --msg "${_msg_dir}/${_name}.msg"
            --out-cpp "${_adapters}/include"
            --out-py "${_adapters}/python"
    DEPENDS "${_msg_dir}/${_name}.msg" ${_flux_gen_sources}
    COMMENT "flux_gen: sensor_msgs/${_name} adapter"
    VERBATIM)
  list(APPEND _generated
    "${_adapters}/include/sensor_msgs/flux/${_snake}.hpp"
    "${_adapters}/python/sensor_msgs_flux/${_snake}.py")
endforeach()

add_custom_target(${PROJECT_NAME}_adapters ALL DEPENDS ${_generated})

install(DIRECTORY "${_adapters}/include/" DESTINATION "include/${PROJECT_NAME}")
# The Python adapter is a runtime import, so it goes on the ament python path.
install(DIRECTORY "${_adapters}/python/"
  DESTINATION "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

ament_export_include_directories("include/${PROJECT_NAME}")
ament_package()
