cmake_minimum_required(VERSION 3.16)
project(flux_gen NONE)

# flux_gen is a pure-Python build-time generator (no compiled artifacts). It installs the
# flux_gen package onto the ament python path, exports flux_generate_adapters() for message
# packages to call, and runs its unit tests with pytest.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(FILES cmake/flux_generate_adapters.cmake DESTINATION share/${PROJECT_NAME}/cmake)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_parse test/test_parse.py)
  ament_add_pytest_test(test_flatten test/test_flatten.py)
  ament_add_pytest_test(test_fingerprint test/test_fingerprint.py)
  ament_add_pytest_test(test_layout test/test_layout.py)
  ament_add_pytest_test(test_emit test/test_emit.py)
endif()

ament_package(CONFIG_EXTRAS cmake/flux_gen-extras.cmake.in)
