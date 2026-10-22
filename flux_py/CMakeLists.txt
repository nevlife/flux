cmake_minimum_required(VERSION 3.16)
project(flux_py VERSION 0.1.0 LANGUAGES CXX)

# flux for rclpy nodes: a nanobind module over the C ABI plus the Python ROS layer.

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
find_package(flux_ffi REQUIRED)
find_package(flux_cpp REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)

# Installed before ament_package(), which is where PYTHON_INSTALL_DIR is otherwise set.
_ament_cmake_python_get_python_install_dir()
_ament_cmake_python_register_environment_hook()

set(NANOBIND_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/nanobind)
if(NOT EXISTS ${NANOBIND_DIR}/CMakeLists.txt)
  message(FATAL_ERROR "thirdparty/nanobind is empty: run `git submodule update --init` in flux")
endif()
add_subdirectory(${NANOBIND_DIR} ${CMAKE_CURRENT_BINARY_DIR}/nanobind EXCLUDE_FROM_ALL)

nanobind_add_module(_flux NB_STATIC src/flux_py.cpp src/wire.cpp)
# numpy's C API makes arrays an order of magnitude faster than nb::ndarray's numpy.array() call.
execute_process(
  COMMAND ${Python_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
  OUTPUT_VARIABLE NUMPY_INCLUDE OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
target_include_directories(_flux SYSTEM PRIVATE ${NUMPY_INCLUDE})
# Only flux_cpp's header-only detail layer is used, so its library and rclcpp are not linked.
get_target_property(FLUX_CPP_INCLUDE flux_cpp::flux_cpp INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(_flux PRIVATE ${FLUX_CPP_INCLUDE})
target_include_directories(_flux SYSTEM PRIVATE ${NANOBIND_DIR}/include)
target_link_libraries(_flux PRIVATE flux::ffi)
target_compile_options(_flux PRIVATE -Wall -Wextra -Wpedantic -Werror)

install(TARGETS _flux LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/flux")
install(FILES flux/__init__.py flux/message_filters.py flux/wire.py DESTINATION "${PYTHON_INSTALL_DIR}/flux")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  add_library(flux_exit_delay MODULE test/exit_delay.cpp)
  add_library(flux_trace_log MODULE test/trace_log.cpp)
  ament_add_pytest_test(test_trace test/test_trace.py TIMEOUT 120
    ENV FLUX_TRACE_LOG_LIB=$<TARGET_FILE:flux_trace_log>)
  foreach(t test_channel test_executor test_partitioned test_message_filters test_wire test_emit test_tensor test_gpu
      test_arena)
    ament_add_pytest_test(${t} test/${t}.py TIMEOUT 120
      ENV FLUX_EXIT_DELAY=$<TARGET_FILE:flux_exit_delay>)
  endforeach()
  # Again on the parker threads a kernel below 5.16 waits with.
  foreach(t test_executor test_partitioned)
    ament_add_pytest_test(${t}_parked test/${t}.py TIMEOUT 120
      ENV FLUX_EXIT_DELAY=$<TARGET_FILE:flux_exit_delay> FLUX_DISABLE_FUTEX_WAITV=1)
  endforeach()
endif()

ament_package()
