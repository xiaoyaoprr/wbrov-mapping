cmake_minimum_required(VERSION 3.16)
project(wbrov_message_info LANGUAGES CXX)

# Portable packaging definition, added during the 2026-10-15 source export.
# The archived production command used C++17, -fPIC, -O2 and rcl/rcutils/rmw.
# This definition has not been rebuilt on the target as part of the export.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_SKIP_RPATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)

add_library(wbrov_message_info SHARED message_info_adapter.cpp)
target_compile_options(wbrov_message_info PRIVATE -O2)
ament_target_dependencies(wbrov_message_info rcl rcutils rmw)
install(TARGETS wbrov_message_info LIBRARY DESTINATION lib)
