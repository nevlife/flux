cmake_minimum_required(VERSION 3.16)
project(flux2_ffi VERSION 0.1.0 LANGUAGES NONE)

# cargo builds the library from the workspace one level up. Plain CMake, not ament: the C ABI has
# no ROS dependency. Installs the shared and static library, the headers and find_package(flux2_ffi).

set(cargo_home $ENV{CARGO_HOME})
if(NOT cargo_home)
  set(cargo_home $ENV{HOME}/.cargo)
endif()
find_program(CARGO cargo PATHS ${cargo_home}/bin NO_DEFAULT_PATH)
if(NOT CARGO)
  message(FATAL_ERROR "rustup's cargo not found under ${cargo_home}/bin: install rustup")
endif()

set(WORKSPACE ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(CARGO_DIR ${CMAKE_CURRENT_BINARY_DIR}/cargo)

# Always runs; cargo decides what is out of date. MAKEFLAGS and MFLAGS name make's jobserver pipes,
# which a custom target's command does not inherit.
add_custom_target(flux2_c_cargo ALL
  COMMAND ${CMAKE_COMMAND} -E env --unset=MAKEFLAGS --unset=MFLAGS
    CARGO_TARGET_DIR=${CARGO_DIR} PATH=${cargo_home}/bin:$ENV{PATH}
    ${CARGO} build --release --locked -p flux2_ffi
  WORKING_DIRECTORY ${WORKSPACE}
  USES_TERMINAL
  COMMENT "cargo build flux2_ffi")

# The soname is libflux2_ffi.so.<FLUX2_ABI_VERSION> (build.rs); libflux2_ffi.so links to it.
file(STRINGS include/flux2.h abi REGEX "^#define FLUX2_ABI_VERSION ")
# Read at configure time, so a new ABI in the header has to configure again.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS include/flux2.h)
string(REGEX REPLACE ".* " "" abi "${abi}")
set(soname libflux2_ffi.so.${abi})
file(CREATE_LINK ${soname} ${CMAKE_CURRENT_BINARY_DIR}/libflux2_ffi.so SYMBOLIC)
install(FILES ${CARGO_DIR}/release/libflux2_ffi.so DESTINATION lib RENAME ${soname})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libflux2_ffi.so ${CARGO_DIR}/release/libflux2_ffi.a
  DESTINATION lib)
install(FILES include/flux2.h DESTINATION include)
install(FILES include/flux2/trace.h DESTINATION include/flux2)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/flux2_ffiConfigVersion.cmake
  COMPATIBILITY SameMinorVersion)
install(FILES cmake/flux2_ffiConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/flux2_ffiConfigVersion.cmake
  DESTINATION share/flux2_ffi/cmake)
