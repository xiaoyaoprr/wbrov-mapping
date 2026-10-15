cmake_minimum_required(VERSION 3.16)
project(wbrov_header_health_observer_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  find_library(
    builtin_interfaces__rosidl_generator_c_LIB
    NAMES builtin_interfaces__rosidl_generator_c
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(header_health_observer header_health_observer.cpp)
ament_target_dependencies(header_health_observer SYSTEM
  builtin_interfaces nav_msgs rclcpp rtabmap_msgs sensor_msgs)
target_compile_options(header_health_observer PRIVATE
  -Wall -Wextra -Wpedantic -Werror)

add_executable(header_health_fixture header_health_fixture.cpp)
ament_target_dependencies(header_health_fixture SYSTEM
  builtin_interfaces nav_msgs rclcpp rtabmap_msgs sensor_msgs)
target_compile_options(header_health_fixture PRIVATE
  -Wall -Wextra -Wpedantic -Werror)

install(TARGETS header_health_observer header_health_fixture
  RUNTIME DESTINATION bin)
