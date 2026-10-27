cmake_minimum_required(VERSION 3.16)
project(flux_cli VERSION 0.1.0 LANGUAGES NONE)

# cargo builds the flux command from the workspace one level up; installed under bin, which the
# workspace's setup script puts on PATH. Plain CMake, not ament: it needs no ROS.

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
add_custom_target(flux_cli_cargo ALL
  COMMAND ${CMAKE_COMMAND} -E env --unset=MAKEFLAGS --unset=MFLAGS
    CARGO_TARGET_DIR=${CARGO_DIR} PATH=${cargo_home}/bin:$ENV{PATH}
    ${CARGO} build --release --locked -p flux_cli
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
  USES_TERMINAL
  COMMENT "cargo build flux_cli")

install(PROGRAMS ${CARGO_DIR}/release/flux DESTINATION bin)
install(FILES environment/flux-complete.bash DESTINATION share/${PROJECT_NAME}/environment)
