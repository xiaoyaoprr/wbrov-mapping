cmake_minimum_required(VERSION 3.8)
project(wbrov_frontend_rate_ab_candidate_v4)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(Threads REQUIRED)
find_library(ZSTD_LIBRARY NAMES zstd REQUIRED)
find_library(WBROV_ZSTD_STATIC_LIBRARY NAMES libzstd.a REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(ZED 4 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

add_executable(zed_rgbd_publisher_rate_ab zed_rgbd_dual_stream_publisher.cpp)
target_compile_options(zed_rgbd_publisher_rate_ab PRIVATE -Wall -Wextra -Werror)
target_include_directories(
  zed_rgbd_publisher_rate_ab SYSTEM PRIVATE
  ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(zed_rgbd_publisher_rate_ab nav_msgs rclcpp sensor_msgs rtabmap_msgs)
target_link_libraries(
  zed_rgbd_publisher_rate_ab
  ${WBROV_ZSTD_STATIC_LIBRARY}
  ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${OpenCV_LIBS} Threads::Threads)
# The ZED SDK exports an older ZSTD API, without ZSTD_compress2. Dynamic
# linking can therefore create a context in the SDK and use it in system
# ZSTD_compress2, corrupting memory on the first full RGB-D frame. Keep our
# complete codec private; SDK calls must continue to resolve within the SDK.
set_property(TARGET zed_rgbd_publisher_rate_ab APPEND_STRING PROPERTY
  LINK_FLAGS " -Wl,--exclude-libs,libzstd.a")

enable_testing()
add_executable(rgbd_frame_codec_test rgbd_frame_codec_test.cpp)
target_compile_options(rgbd_frame_codec_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(rgbd_frame_codec_test ${ZSTD_LIBRARY})
add_test(NAME rgbd_frame_codec_offline COMMAND rgbd_frame_codec_test)
add_executable(rgbd_codec_sdk_test rgbd_codec_sdk_test.cpp)
target_compile_options(rgbd_codec_sdk_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_include_directories(rgbd_codec_sdk_test SYSTEM PRIVATE ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
target_link_libraries(rgbd_codec_sdk_test ${WBROV_ZSTD_STATIC_LIBRARY} ${ZED_LIBRARIES} ${CMAKE_DL_LIBS})
set_property(TARGET rgbd_codec_sdk_test APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--exclude-libs,libzstd.a")
add_test(NAME rgbd_codec_sdk_offline COMMAND rgbd_codec_sdk_test)
set_tests_properties(rgbd_codec_sdk_offline PROPERTIES TIMEOUT 30)
add_executable(rgbd_spool_test rgbd_spool_test.cpp)
target_compile_options(rgbd_spool_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(rgbd_spool_test Threads::Threads ${ZSTD_LIBRARY})
add_test(NAME rgbd_spool_offline COMMAND rgbd_spool_test)
add_executable(async_diagnostics_test async_diagnostics_test.cpp)
target_compile_options(async_diagnostics_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(async_diagnostics_test Threads::Threads)
add_test(NAME async_diagnostics_offline COMMAND async_diagnostics_test)
set_tests_properties(async_diagnostics_offline PROPERTIES TIMEOUT 10)
add_executable(bounded_async_writer_test bounded_async_writer_test.cpp)
target_compile_options(bounded_async_writer_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(bounded_async_writer_test Threads::Threads)
add_test(NAME bounded_async_writer_offline COMMAND bounded_async_writer_test)
set_tests_properties(bounded_async_writer_offline PROPERTIES TIMEOUT 10)

add_executable(wbrov_verify_svo verify_svo.cpp)
target_compile_options(wbrov_verify_svo PRIVATE -Wall -Wextra -Werror)
target_include_directories(wbrov_verify_svo SYSTEM PRIVATE ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
target_link_libraries(wbrov_verify_svo ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})

add_executable(raw_calibration_audit_test raw_calibration_audit_test.cpp)
target_compile_options(raw_calibration_audit_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_include_directories(raw_calibration_audit_test PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(raw_calibration_audit_test ${OpenCV_LIBS})
add_test(NAME raw_calibration_audit_offline COMMAND raw_calibration_audit_test)

foreach(test_name IN ITEMS
  depth_range_filter
  dual_stream_schedule
  exact_odom_stamp_gate
  map_frame_lifecycle
  map_stamp_audit
  valid_odom_pose
  imu_frame_conversion
  capture_telemetry)
  add_executable(${test_name}_test ${test_name}_test.cpp)
  target_compile_options(${test_name}_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
  add_test(NAME ${test_name}_offline COMMAND ${test_name}_test)
endforeach()

# Isolated build candidate only: deliberately no install/export rules.
