cmake_minimum_required(VERSION 3.14)
project(flux_cli NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Installed to bin, not lib/${PROJECT_NAME}: this is a command a person types, not something
# `ros2 run` launches. flux_cli deliberately needs no ROS, so putting it behind `ros2 run` would
# make the one tool that works without ROS reachable only through it. A plain script rather than
# a setuptools console_script keeps the package on ament_cmake like every other one here.
install(PROGRAMS scripts/flux DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_flux_cli test)
endif()

ament_package()
