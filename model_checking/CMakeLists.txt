cmake_minimum_required(VERSION 3.16)
project(flux2_model_checking NONE)

# Nothing to build: this package puts the harnesses under `colcon test`. Plain CMake, no ROS. The
# SC models take about half a minute, the Miri tests about twenty seconds. A checker that is
# missing is said so at configure time, never skipped in silence.
include(CTest)
if(NOT BUILD_TESTING)
  return()
endif()

set(here ${CMAKE_CURRENT_SOURCE_DIR})
foreach(harness IN ITEMS readers wake sweep)
  foreach(target IN ITEMS check bug vacuous configs)
    add_test(NAME model_${harness}_${target} COMMAND make -s -C ${here}/${harness} ${target})
    set_tests_properties(model_${harness}_${target} PROPERTIES TIMEOUT 600)
  endforeach()
endforeach()

add_test(NAME litmus_native COMMAND make -s -C ${here}/litmus native)

find_program(GENMC genmc HINTS $ENV{HOME}/.local/bin)
if(GENMC)
  message(STATUS "genmc: ${GENMC}")
  foreach(target IN ITEMS check mutate)
    add_test(NAME litmus_${target} COMMAND make -s -C ${here}/litmus ${target} GENMC=${GENMC})
    set_tests_properties(litmus_${target} PROPERTIES TIMEOUT 1800)
  endforeach()
else()
  message(WARNING "genmc not found: the RC11 litmus tests are not registered")
endif()

# The nightly the fuzz crate pins, which scripts/miri.sh uses too.
file(STRINGS ${here}/../fuzz/rust-toolchain.toml channel REGEX "^channel")
string(REGEX REPLACE "^channel = \"(.*)\"$" "\\1" toolchain "${channel}")
set(cargo_home $ENV{CARGO_HOME})
if(NOT cargo_home)
  set(cargo_home $ENV{HOME}/.cargo)
endif()
execute_process(COMMAND ${cargo_home}/bin/cargo +${toolchain} miri --version
  RESULT_VARIABLE no_miri OUTPUT_QUIET ERROR_QUIET)
if(no_miri EQUAL 0)
  add_test(NAME miri_orderings COMMAND ${here}/../scripts/miri.sh)
  set_tests_properties(miri_orderings PROPERTIES TIMEOUT 1800)
else()
  message(WARNING "miri missing for ${toolchain}: the ordering tests are not registered")
endif()
