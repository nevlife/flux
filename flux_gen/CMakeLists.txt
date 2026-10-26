cmake_minimum_required(VERSION 3.16)
project(flux_gen NONE)

# The generator is a Rust program cargo builds from the workspace one level up. Installs it under
# lib/flux_gen and exports flux_generate_adapters() for message packages to call. Its tests are
# cargo's; the emitted code is tested where its runtimes are (flux_py test_emit).

find_package(ament_cmake REQUIRED)

set(cargo_home $ENV{CARGO_HOME})
if(NOT cargo_home)
  set(cargo_home $ENV{HOME}/.cargo)
endif()
find_program(CARGO cargo PATHS ${cargo_home}/bin NO_DEFAULT_PATH)
if(NOT CARGO)
  message(FATAL_ERROR "rustup's cargo not found under ${cargo_home}/bin: install rustup")
endif()

set(CARGO_DIR ${CMAKE_CURRENT_BINARY_DIR}/cargo)

# Always runs; cargo decides what is out of date. MAKEFLAGS and MFLAGS name make's jobserver pipes,
# which a custom target's command does not inherit.
add_custom_target(flux_gen_cargo ALL
  COMMAND ${CMAKE_COMMAND} -E env --unset=MAKEFLAGS --unset=MFLAGS
    CARGO_TARGET_DIR=${CARGO_DIR} PATH=${cargo_home}/bin:$ENV{PATH}
    ${CARGO} build --release --locked -p flux_gen
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
  USES_TERMINAL
  COMMENT "cargo build flux_gen")

install(PROGRAMS ${CARGO_DIR}/release/flux_gen DESTINATION lib/${PROJECT_NAME})
install(FILES cmake/flux_generate_adapters.cmake DESTINATION share/${PROJECT_NAME}/cmake)

ament_package(CONFIG_EXTRAS cmake/flux_gen-extras.cmake.in)
