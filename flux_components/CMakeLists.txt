cmake_minimum_required(VERSION 3.16)
project(flux_components VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(flux_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# Shared, so a container and the subclasses built on it hold one copy of flux_cpp's executors.
add_library(flux_components SHARED src/component_manager.cpp)
target_compile_options(flux_components PRIVATE -Wall -Wextra -Werror)
target_include_directories(flux_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(flux_components PUBLIC
  flux_cpp::flux_cpp rclcpp::rclcpp rclcpp_components::component_manager)

add_executable(component_container src/component_container.cpp)
target_compile_options(component_container PRIVATE -Wall -Wextra -Werror)
target_link_libraries(component_container flux_components)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS flux_components EXPORT flux_componentsTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS component_container DESTINATION lib/${PROJECT_NAME})
ament_export_targets(flux_componentsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(flux_cpp rclcpp rclcpp_components)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(composition_interfaces REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  # The test binary and the components it loads record into one shared library.
  add_library(flux_components_test_probe SHARED test/probe.cpp)
  add_library(flux_components_test_nodes SHARED test/components.cpp)
  target_link_libraries(flux_components_test_nodes
    flux_components_test_probe flux_cpp::flux_cpp rclcpp::rclcpp rclcpp_components::component)
  ament_add_gtest(test_component_container test/test_component_container.cpp TIMEOUT 120)
  target_link_libraries(test_component_container flux_components flux_components_test_probe
    ${composition_interfaces_TARGETS} ${rosgraph_msgs_TARGETS})
  target_compile_definitions(test_component_container PRIVATE
    FLUX_TEST_COMPONENTS="$<TARGET_FILE:flux_components_test_nodes>"
    FLUX_CONTAINER="$<TARGET_FILE:component_container>")
  add_dependencies(test_component_container flux_components_test_nodes component_container)
  foreach(t flux_components_test_probe flux_components_test_nodes test_component_container)
    target_compile_options(${t} PRIVATE -Wall -Wextra -Werror)
  endforeach()
endif()

ament_package()
