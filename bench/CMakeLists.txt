cmake_minimum_required(VERSION 3.20)

# The calibration's bench: reads the driver's pose recordings and runs the overlay's own calibration code over them. It is a
# project of its own, not part of the app's build, and needs the app's submodules (openvr, fmt, glaze) checked out:
#
#   cmake -S bench -B build-bench -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build-bench && ctest --test-dir build-bench
project(SpaceCalibratorBench CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(APP ${CMAKE_CURRENT_SOURCE_DIR}/..)

# the recording's format, reader and writer: the driver's own
add_library(recording STATIC ${APP}/src/common/pose_recording.cpp)
target_include_directories(recording PUBLIC ${APP}/src/common)

# the overlay's calibration, as the overlay builds it but for the headers in stubs/, which come first
add_library(overlay_calibration STATIC ${APP}/src/overlay/calibration.cpp ${APP}/src/overlay/mount_estimator.cpp)
target_include_directories(overlay_calibration PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${APP}/src/overlay
    ${APP}/src/common
    ${APP}/vendor
    ${APP}/vendor/openvr/headers
    ${APP}/vendor/fmt/include
    ${APP}/vendor/glaze/include
    ${APP}/vendor/eigen)
target_compile_definitions(overlay_calibration PUBLIC IS_OVERLAY EIGEN_HAS_STD_RESULT_OF=0)

# what the driver applies of a calibration, as the driver builds it but for OpenVR's types, which it takes as the overlay has them
add_library(driver_calibration STATIC ${APP}/src/driver/applied_calibration.cpp)
target_include_directories(driver_calibration PUBLIC ${APP}/src/driver)
target_link_libraries(driver_calibration PUBLIC overlay_calibration)

add_executable(spacecal-bench
    src/main.cpp
    src/session.cpp
    src/reference.cpp
    src/simulate.cpp
    src/replay.cpp
    src/replay_driver.cpp)
target_include_directories(spacecal-bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(spacecal-bench PRIVATE recording overlay_calibration driver_calibration)
if(NOT MSVC)
    target_compile_options(spacecal-bench PRIVATE -Wall -Wextra)
    # a build that runs on a machine other than the one it was made on
    target_link_options(spacecal-bench PRIVATE -static-libstdc++ -static-libgcc)
endif()

enable_testing()
add_executable(test_recording tests/test_recording.cpp)
target_link_libraries(test_recording PRIVATE recording)
find_package(Threads REQUIRED)
target_link_libraries(test_recording PRIVATE Threads::Threads)
add_test(NAME recording COMMAND test_recording)

# the driver's recorder as the driver builds it, against a stand-in for SteamVR
add_executable(test_session_recorder tests/test_session_recorder.cpp ${APP}/src/driver/session_recorder.cpp)
target_include_directories(test_session_recorder PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${APP}/src/driver
    ${APP}/src/common
    ${APP}/vendor
    ${APP}/vendor/openvr/headers
    ${APP}/vendor/fmt/include
    ${APP}/vendor/eigen)
target_compile_definitions(test_session_recorder PRIVATE IS_DRIVER EIGEN_HAS_STD_RESULT_OF=0)
target_link_libraries(test_session_recorder PRIVATE recording Threads::Threads)
add_test(NAME session_recorder COMMAND test_session_recorder)

# the mount estimator on made-up motions
add_executable(test_mount_estimator tests/test_mount_estimator.cpp)
target_link_libraries(test_mount_estimator PRIVATE overlay_calibration)
add_test(NAME mount_estimator COMMAND test_mount_estimator)

# the slots the driver and the overlay share their state through, from many threads and two processes; built once with the
# overlay's copy of OpenVR's DriverPose_t and once with the driver's own, which have to be laid out the same
foreach(side IN ITEMS OVERLAY DRIVER)
    string(TOLOWER ${side} side_name)
    add_executable(test_shared_state_${side_name} tests/test_shared_state.cpp)
    target_include_directories(test_shared_state_${side_name} PRIVATE ${APP}/src/common ${APP}/vendor/openvr/headers)
    target_compile_definitions(test_shared_state_${side_name} PRIVATE IS_${side})
    target_link_libraries(test_shared_state_${side_name} PRIVATE Threads::Threads)
    add_test(NAME shared_state_${side_name} COMMAND test_shared_state_${side_name})
endforeach()

# what the driver applies of a calibration, on made-up poses
add_executable(test_applied_calibration tests/test_applied_calibration.cpp)
target_link_libraries(test_applied_calibration PRIVATE driver_calibration)
add_test(NAME applied_calibration COMMAND test_applied_calibration)
add_test(NAME bench COMMAND ${CMAKE_COMMAND} -DBENCH=$<TARGET_FILE:spacecal-bench> -DWORK=${CMAKE_CURRENT_BINARY_DIR} -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_bench.cmake)
