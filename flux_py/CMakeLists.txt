cmake_minimum_required(VERSION 3.16)
project(flux_py VERSION 0.0.0 LANGUAGES CXX)

# flux_py is the nanobind Python binding over flux_core. nanobind is vendored (no system
# dependency) and pulled in with add_subdirectory, mirroring how mature C++ cores embed it.

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
find_package(ament_cmake_python REQUIRED)
find_package(flux_core REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)  # ament's PYTHON_INSTALL_DIR helper
find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)  # nanobind

# PYTHON_INSTALL_DIR is only populated during ament_package(); we install before that, so
# compute it now (same helper ament uses, so the install path and PYTHONPATH hook agree).
_ament_cmake_python_get_python_install_dir()

# Vendored nanobind lives at the flux repo root (shared, single submodule).
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/nanobind
                 ${CMAKE_CURRENT_BINARY_DIR}/nanobind EXCLUDE_FROM_ALL)

nanobind_add_module(_flux NB_STATIC src/flux_py.cpp)
# Link the imported target directly (keyword signature, matching nanobind). It carries
# flux_core's include dirs too. ament_target_dependencies (plain signature) would clash.
target_link_libraries(_flux PRIVATE flux_core::flux_core)

# Install the extension and the thin Python package into the ament python path so that
# `import flux` works after sourcing the workspace.
install(TARGETS _flux LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/flux")
install(FILES flux/__init__.py DESTINATION "${PYTHON_INSTALL_DIR}/flux")
install(FILES flux/ros/__init__.py flux/ros/message_filters.py
        DESTINATION "${PYTHON_INSTALL_DIR}/flux/ros")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_zero_copy test/test_zero_copy.py)
  ament_add_pytest_test(test_executor test/test_executor.py)
  ament_add_pytest_test(test_qos test/test_qos.py)
  ament_add_pytest_test(test_memory_policy test/test_memory_policy.py)
  ament_add_pytest_test(test_ros_executor test/test_ros_executor.py)
  ament_add_pytest_test(test_ros_partitioned test/test_ros_partitioned.py)
  ament_add_pytest_test(test_ros_message_filters test/test_ros_message_filters.py)
  ament_add_pytest_test(test_rt test/test_rt.py)
  ament_add_pytest_test(test_doc_examples test/test_doc_examples.py)
  ament_add_pytest_test(test_doc_adapter test/test_doc_adapter.py)
  ament_add_pytest_test(test_gpu test/test_gpu.py)
  # Compiles a C++ driver and runs it against a Python process, so it needs longer than
  # the 60s default and libflux_core from this same build.
  ament_add_pytest_test(test_cross_language test/test_cross_language.py TIMEOUT 180)
endif()

ament_package()
