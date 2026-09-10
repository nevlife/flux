cmake_minimum_required(VERSION 3.16)
project(flux_cpp VERSION 0.0.0 LANGUAGES CXX)

# flux_cpp is the ROS 2 wrapper over flux_core. Unlike flux_core it is not standalone --
# it needs rclcpp, so it only builds as an ament_cmake package.

# colcon passes no build type; without this the package compiles at -O0 (flux_core/CMakeLists.txt).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(flux_core REQUIRED)
find_package(rclcpp REQUIRED)
# Header-only: flux/ros/message_filters/subscriber.hpp is the only thing that uses it, and
# nothing in this library compiles it. Required rather than optional because that header ships
# in the public include directory, and a public header that compiles or not depending on what
# else is installed is worse than one small dependency.
find_package(message_filters REQUIRED)
# Only the chain-declaration reader needs it. flux_core stays standalone; parsing a file is this
# layer's work.
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)  # the vendor package only points at it; the target comes from here

add_library(flux_cpp
  src/ros_boundary.cpp
  src/publisher.cpp
  src/subscription.cpp
  src/executor.cpp
  src/partitioned_executor.cpp
  src/rt_spec.cpp)
add_library(flux::cpp ALIAS flux_cpp)
# Static, but linked into shared objects (the rviz2 plugin in flux_tools), same as flux_core.
set_target_properties(flux_cpp PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(flux_cpp PUBLIC cxx_std_17)
target_include_directories(flux_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(flux_cpp PUBLIC flux_core rclcpp)
# Private: yaml-cpp is an implementation detail of the chain-declaration reader. Its types never
# reach flux/ros/rt_spec.hpp, so a consumer of flux_cpp does not inherit the dependency.
target_link_libraries(flux_cpp PRIVATE yaml-cpp::yaml-cpp)
target_compile_options(flux_cpp PRIVATE -Wall -Wextra -Wpedantic)

# The code fences in docs/en/api.en.md section 3 are the anchored regions of this file. Compiling it
# is what stops a renamed or re-signatured API from outliving the document; the other direction
# (fence still equals region) is scripts/check_doc_examples.py, wired in as a test below.
# Deliberately not installed: nothing links it and nothing runs it.
add_library(flux_cpp_doc_examples STATIC examples/doc_api.cpp)
target_link_libraries(flux_cpp_doc_examples flux_cpp)
ament_target_dependencies(flux_cpp_doc_examples rclcpp)
target_compile_options(flux_cpp_doc_examples PRIVATE -Wall -Wextra -Wpedantic)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS flux_cpp
  EXPORT export_flux_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
ament_export_targets(export_flux_cpp HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
# flux_cpp is a static library, so even a PRIVATE link lands in the exported interface: a
# consumer does the final link and needs the archive's dependencies resolvable. Export the two
# packages that make the target findable -- the vendor sets yaml-cpp_DIR, the config defines the
# target. Without this a downstream package fails at configure with "target was not found".
ament_export_dependencies(flux_core message_filters rclcpp yaml_cpp_vendor yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  ament_add_gtest(test_ros_coexist test/test_ros_coexist.cpp)
  if(TARGET test_ros_coexist)
    target_link_libraries(test_ros_coexist flux_cpp)
    ament_target_dependencies(test_ros_coexist rclcpp std_msgs std_srvs)
  endif()

  ament_add_gtest(test_flux_executor test/test_flux_executor.cpp)
  if(TARGET test_flux_executor)
    target_link_libraries(test_flux_executor flux_cpp)
    ament_target_dependencies(test_flux_executor rclcpp std_msgs)
  endif()

  ament_add_gtest(test_pull_surface test/test_pull_surface.cpp)
  if(TARGET test_pull_surface)
    target_link_libraries(test_pull_surface flux_cpp)
    ament_target_dependencies(test_pull_surface rclcpp)
  endif()

  ament_add_gtest(test_rt_spec test/test_rt_spec.cpp)
  if(TARGET test_rt_spec)
    target_link_libraries(test_rt_spec flux_cpp)
    ament_target_dependencies(test_rt_spec rclcpp)
  endif()

  ament_add_gtest(test_partitioned_executor test/test_partitioned_executor.cpp)
  if(TARGET test_partitioned_executor)
    target_link_libraries(test_partitioned_executor flux_cpp)
    ament_target_dependencies(test_partitioned_executor rclcpp std_msgs)
  endif()

  ament_add_gtest(test_rt_chain test/test_rt_chain.cpp)
  if(TARGET test_rt_chain)
    target_link_libraries(test_rt_chain flux_cpp)
    ament_target_dependencies(test_rt_chain rclcpp std_msgs)
  endif()

  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  # Section 2 of docs/en/api.en.md shows generated adapters, so its examples only compile against real
  # generator output. Emitting it here is what puts kFingerprint, Builder-over-WriteSlot, commit()
  # and the bridge pair under a build rather than under review. Test-only: a plain build of
  # flux_cpp must not need flux_gen or sensor_msgs.
  find_package(flux_gen REQUIRED)
  find_package(sensor_msgs REQUIRED)
  set(_doc_adapters "${CMAKE_CURRENT_BINARY_DIR}/doc_adapters")
  set(_doc_cloud_msg "${CMAKE_CURRENT_SOURCE_DIR}/../docs/examples/Cloud.msg")
  # The emitters are an input: renaming what the generator produces has to fail here, and it only
  # does if an edit to them regenerates. Same reason flux_generate_adapters() globs them.
  file(GLOB _flux_gen_sources CONFIGURE_DEPENDS "${FLUX_GEN_PYTHON_DIR}/flux_gen/*.py")
  add_custom_command(
    OUTPUT "${_doc_adapters}/my_pkg/flux/cloud.hpp"
           "${_doc_adapters}/sensor_msgs/flux/image.hpp"
           "${_doc_adapters}/sensor_msgs/flux/image_ros.hpp"
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${FLUX_GEN_PYTHON_DIR}:$ENV{PYTHONPATH}"
            ${Python3_EXECUTABLE} -m flux_gen --pkg my_pkg
            --msg "${_doc_cloud_msg}" --out-cpp "${_doc_adapters}"
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${FLUX_GEN_PYTHON_DIR}:$ENV{PYTHONPATH}"
            ${Python3_EXECUTABLE} -m flux_gen --pkg sensor_msgs
            --msg "${sensor_msgs_DIR}/../msg/Image.msg" --out-cpp "${_doc_adapters}"
    DEPENDS "${_doc_cloud_msg}" ${_flux_gen_sources}
    COMMENT "flux_gen: docs/en/api.en.md section 2 adapters"
    VERBATIM)

  # message_filters integration: needs a real generated adapter with a Header, so it hangs off the
  # same doc_adapters output as the api.md examples.
  ament_add_gtest(test_message_filters test/test_message_filters.cpp
    "${_doc_adapters}/sensor_msgs/flux/image.hpp")
  if(TARGET test_message_filters)
    target_link_libraries(test_message_filters flux_cpp)
    target_include_directories(test_message_filters PRIVATE "${_doc_adapters}")
    ament_target_dependencies(test_message_filters message_filters rclcpp sensor_msgs)
  endif()

  # docs/en/raw_api.en.md: the .msg-less path. Separate target because the document is separately
  # paired in scripts/check_doc_examples.py, and this half needs no generated adapter.
  add_library(flux_cpp_doc_raw_examples STATIC examples/doc_api_raw.cpp)
  target_link_libraries(flux_cpp_doc_raw_examples flux_cpp)
  ament_target_dependencies(flux_cpp_doc_raw_examples rclcpp)
  target_compile_options(flux_cpp_doc_raw_examples PRIVATE -Wall -Wextra -Wpedantic)

  add_library(flux_cpp_doc_adapter_examples STATIC
    examples/doc_api_adapter.cpp
    "${_doc_adapters}/my_pkg/flux/cloud.hpp"
    "${_doc_adapters}/sensor_msgs/flux/image_ros.hpp")
  target_include_directories(flux_cpp_doc_adapter_examples PRIVATE "${_doc_adapters}")
  target_link_libraries(flux_cpp_doc_adapter_examples flux_cpp)
  ament_target_dependencies(flux_cpp_doc_adapter_examples message_filters rclcpp sensor_msgs)
  target_compile_options(flux_cpp_doc_adapter_examples PRIVATE -Wall -Wextra -Wpedantic)

  # Both cover the two languages at once, so they live with the C++ package rather than running
  # twice.
  add_test(NAME check_doc_examples
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/check_doc_examples.py)
  add_test(NAME check_contracts
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/check_contracts.py)
  # The C++ half of the reverse check: flux_py's test suite walks __all__, this walks the headers.
  add_test(NAME check_api_surface
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/check_api_surface.py)
  # Measures all three packages, so it lives here for the same reason the two above do.
  add_test(NAME check_comment_density
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/check_comment_density.py)
endif()

ament_package()
