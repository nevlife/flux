cmake_minimum_required(VERSION 3.16)
project(flux_wrapper_test_msgs)

# Messages for flux_wrapper's tests, and the shape of a message package that builds with and
# without flux: the adapters are generated only where flux is built.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msgs msg/Bar.msg msg/Blob.msg msg/Foo.msg)
rosidl_generate_interfaces(${PROJECT_NAME} ${msgs} DEPENDENCIES geometry_msgs std_msgs)
if("$ENV{ENABLE_FLUX}" STREQUAL "1")
  find_package(flux_gen REQUIRED)
  flux_generate_adapters(${msgs})
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
