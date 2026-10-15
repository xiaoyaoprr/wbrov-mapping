cmake_minimum_required(VERSION 3.16)
project(wbrov_safe_reloc_gid_frontend_native LANGUAGES CXX)

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
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(safe_reloc_gid_frontend safe_reloc_gid_frontend.cpp)
ament_target_dependencies(safe_reloc_gid_frontend SYSTEM
  builtin_interfaces rclcpp rtabmap_msgs std_msgs)
target_link_libraries(safe_reloc_gid_frontend OpenSSL::Crypto)
target_compile_options(safe_reloc_gid_frontend PRIVATE
  -Wall -Wextra -Wpedantic -Werror)

add_executable(safe_reloc_gid_fixture safe_reloc_gid_fixture.cpp)
ament_target_dependencies(safe_reloc_gid_fixture SYSTEM
  builtin_interfaces rclcpp rtabmap_msgs std_msgs)
target_compile_options(safe_reloc_gid_fixture PRIVATE
  -Wall -Wextra -Wpedantic -Werror)

install(TARGETS safe_reloc_gid_frontend safe_reloc_gid_fixture
  RUNTIME DESTINATION bin)

