cmake_minimum_required(VERSION 3.16)
project(wbrov_deferred_mapping LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(RTABMap REQUIRED)
find_package(ZED 4 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)
find_package(Threads REQUIRED)
find_library(ZSTD_LIBRARY NAMES zstd REQUIRED)
add_executable(wbrov_replay_worker replay_worker.cpp)
target_compile_options(wbrov_replay_worker PRIVATE -Wall -Wextra)
target_include_directories(wbrov_replay_worker BEFORE PRIVATE
  ../rtabmap_core/corelib/include ../rtabmap_ros/rtabmap_slam/include ../zed_rgbd_publisher)
target_include_directories(wbrov_replay_worker SYSTEM PRIVATE ${RTABMap_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(wbrov_replay_worker rtabmap::core OpenSSL::Crypto Threads::Threads ${ZSTD_LIBRARY} ${ZED_LIBRARIES}
  ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${OpenCV_LIBS})
add_executable(wbrov_extract_recorded_rgbd extract_recorded_rgbd.cpp)
target_include_directories(wbrov_extract_recorded_rgbd PRIVATE ../zed_rgbd_publisher)
target_include_directories(wbrov_extract_recorded_rgbd SYSTEM PRIVATE ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
target_link_libraries(wbrov_extract_recorded_rgbd ${ZSTD_LIBRARY} ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} pthread)
# Candidate only. Do not install over the managed realtime mapping runtime.
