cmake_minimum_required(VERSION 3.16)
project(flux_core VERSION 0.0.0 LANGUAGES CXX)

# flux_core is the pure C++17 shared-memory engine. It builds two ways:
#   - standalone:  cmake -S . -B build   (ROS-free; exports find_package(flux_core))
#   - colcon:      built as an ament_cmake package inside a ROS 2 workspace
# The target definition is shared; only the install/export tail branches on ament.

# colcon passes no build type, so without this the engine compiles at -O0: measured, the same
# publish loop costs 5.8% of a core at -O2 and 63.2% at -O0. Every latency claim in docs/ would
# be measured against a build nobody should ship. RelWithDebInfo rather than Release because this
# project's risk is concurrency and the sanitizer runs it leans on need symbols. An explicit
# -DCMAKE_BUILD_TYPE still wins, which is how the sanitizer builds pick their own.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake QUIET)
find_package(Threads REQUIRED)
include(GNUInstallDirs)

# Sanitizer builds, standalone use: cmake -DFLUX_SANITIZER=thread|address|undefined|address,undefined.
# TSan needs the suppressions file for the seqlock idiom's by-design plain reads:
#   TSAN_OPTIONS="suppressions=<src>/test/tsan.supp" ./test_flux_core
set(FLUX_SANITIZER "" CACHE STRING "Compile with -fsanitize=<value> (empty = off)")
if(FLUX_SANITIZER)
  add_compile_options(-fsanitize=${FLUX_SANITIZER} -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=${FLUX_SANITIZER})
endif()

# io_uring's futex opcode is an enum constant, not a macro, so the source cannot test for it.
# Probe by compiling: without it flux_core still builds (Jetson Orin ships 5.15) and callers fall
# back to a thread per channel.
include(CheckCXXSourceCompiles)
check_cxx_source_compiles("
#include <linux/futex.h>
#include <linux/io_uring.h>
int main() { return (int)IORING_OP_FUTEX_WAIT + (int)FUTEX2_SIZE_U32; }
" FLUX_HAS_IO_URING_FUTEX)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/flux/version.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/flux/version.hpp
  @ONLY)

add_library(flux_core
  src/version.cpp
  src/channel.cpp
  src/segment.cpp
  src/discovery.cpp
  src/owner.cpp
  src/rt.cpp
  src/gpu_platform.cpp
  src/gpu_vmm.cpp
  src/io_uring_waiter.cpp
  src/spin_control.cpp
  src/executor.cpp)

# shm_open/shm_unlink live in libc on glibc >= 2.34 (Humble/Jazzy), but link librt
# when present so older toolchains resolve them too.
find_library(FLUX_RT_LIB rt)
if(FLUX_RT_LIB)
  target_link_libraries(flux_core PRIVATE ${FLUX_RT_LIB})
endif()
# The CUDA driver is dlopen'd, never linked, so one binary serves hosts
# with and without a GPU.
target_link_libraries(flux_core PRIVATE ${CMAKE_DL_LIBS})
add_library(flux::core ALIAS flux_core)
# PIC so this static lib can be linked into shared objects (flux_py's nanobind module).
set_target_properties(flux_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(flux_core PUBLIC cxx_std_17)
target_include_directories(flux_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_options(flux_core PRIVATE -Wall -Wextra -Wpedantic)
if(FLUX_HAS_IO_URING_FUTEX)
  target_compile_definitions(flux_core PRIVATE FLUX_HAS_IO_URING_FUTEX=1)
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        PATTERN "*.in" EXCLUDE)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(flux_core_test_srcs
  test/unit/test_smoke.cpp
  test/unit/test_segment_layout.cpp
  test/unit/test_wire.cpp
  test/unit/test_channel.cpp
  test/unit/test_wake.cpp
  test/unit/test_spin_control.cpp
  test/unit/test_owner.cpp
  test/unit/test_domain.cpp
  test/unit/test_rt.cpp
  test/unit/test_gpu_platform.cpp
  test/integration/test_shm.cpp
  test/integration/test_io_uring_waiter.cpp
  test/integration/test_crash_reclaim.cpp
  test/integration/test_multi_publisher.cpp
  test/integration/test_executor.cpp
  test/integration/test_page_commit.cpp
  test/integration/test_discovery_orphan.cpp
  test/integration/test_enumerate.cpp
  test/integration/test_sweep_concurrent.cpp
  test/integration/test_syscall_budget.cpp
  test/integration/test_gpu_vmm.cpp
  test/integration/test_gpu_host_register.cpp)

# Fuzz harnesses for the two trust boundaries a hostile or corrupt peer reaches: the frame parser
# (wire::Reader) and the consumer path over a segment another process wrote. Both are plain
# LLVMFuzzerTestOneInput, so they build two ways:
#   FLUX_FUZZER=ON   real fuzzing, clang only: cmake -DFLUX_FUZZER=ON -DCMAKE_CXX_COMPILER=clang++
#   default          linked against a replay main() and run over a fixed corpus by ctest, so the
#                    harnesses cannot rot between fuzzing sessions
set(FLUX_FUZZER OFF CACHE BOOL "Build the fuzz harnesses with libFuzzer (clang only)")
set(flux_core_fuzzers fuzz_wire fuzz_frame)
function(flux_add_fuzzer name)
  if(FLUX_FUZZER)
    add_executable(${name} test/fuzz/${name}.cpp)
    target_compile_options(${name} PRIVATE -fsanitize=fuzzer,address,undefined -g)
    target_link_options(${name} PRIVATE -fsanitize=fuzzer,address,undefined)
  else()
    add_executable(${name} test/fuzz/${name}.cpp test/fuzz/fuzz_replay_main.cpp)
  endif()
  target_link_libraries(${name} PRIVATE flux_core Threads::Threads)
endfunction()

if(ament_cmake_FOUND)
  install(TARGETS flux_core
    EXPORT export_flux_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)
  ament_export_targets(export_flux_core HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)

  if(BUILD_TESTING)
    # Compile a discovery.cpp test seam (the bootstrap hook) into flux_core for test builds only;
    # a release build (BUILD_TESTING OFF) never defines FLUX_TESTING, so the hook is absent.
    target_compile_definitions(flux_core PRIVATE FLUX_TESTING=1)
    # docs/en/core_api.en.md shows flux_core with no ROS. Linking flux_core alone is what makes that
    # claim testable: an rclcpp name reaching this file fails to link.
    add_library(flux_core_doc_examples STATIC examples/doc_api_core.cpp)
    target_link_libraries(flux_core_doc_examples PRIVATE flux_core)
    target_compile_options(flux_core_doc_examples PRIVATE -Wall -Wextra -Wpedantic)

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_flux_core ${flux_core_test_srcs})
    if(TARGET test_flux_core)
      target_link_libraries(test_flux_core flux_core Threads::Threads)
      target_include_directories(test_flux_core PRIVATE test)
    endif()
    foreach(f IN LISTS flux_core_fuzzers)
      flux_add_fuzzer(${f})
      if(NOT FLUX_FUZZER)  # a libFuzzer binary would fuzz forever; only the replay build is a test
        add_test(NAME ${f}_corpus COMMAND ${f})
      endif()
    endforeach()
  endif()

  ament_package()
else()
  include(CMakePackageConfigHelpers)
  install(TARGETS flux_core
    EXPORT flux_coreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(EXPORT flux_coreTargets
    FILE flux_coreTargets.cmake
    NAMESPACE flux::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/flux_core)
  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/flux_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/flux_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/flux_core)
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/flux_coreConfigVersion.cmake
    COMPATIBILITY SameMajorVersion)
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/flux_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/flux_coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/flux_core)

  include(CTest)
  if(BUILD_TESTING)
    # See the ament branch: FLUX_TESTING compiles the discovery.cpp bootstrap hook, test builds only.
    target_compile_definitions(flux_core PRIVATE FLUX_TESTING=1)
    # docs/en/core_api.en.md shows flux_core with no ROS. Linking flux_core alone is what makes that
    # claim testable: an rclcpp name reaching this file fails to link.
    add_library(flux_core_doc_examples STATIC examples/doc_api_core.cpp)
    target_link_libraries(flux_core_doc_examples PRIVATE flux_core)
    target_compile_options(flux_core_doc_examples PRIVATE -Wall -Wextra -Wpedantic)

    find_package(GTest REQUIRED)
    add_executable(test_flux_core ${flux_core_test_srcs})
    target_link_libraries(test_flux_core PRIVATE flux_core GTest::gtest GTest::gtest_main Threads::Threads)
    target_include_directories(test_flux_core PRIVATE test)
    add_test(NAME test_flux_core COMMAND test_flux_core)
    foreach(f IN LISTS flux_core_fuzzers)
      flux_add_fuzzer(${f})
      if(NOT FLUX_FUZZER)  # a libFuzzer binary would fuzz forever; only the replay build is a test
        add_test(NAME ${f}_corpus COMMAND ${f})
      endif()
    endforeach()
  endif()
endif()
