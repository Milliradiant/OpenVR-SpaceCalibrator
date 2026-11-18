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
add_library(overlay_calibration STATIC ${APP}/src/overlay/calibration.cpp)
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

add_executable(spacecal-bench
    src/main.cpp
    src/session.cpp
    src/reference.cpp
    src/simulate.cpp
    src/replay.cpp)
target_include_directories(spacecal-bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(spacecal-bench PRIVATE recording overlay_calibration)
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
add_test(NAME bench COMMAND ${CMAKE_COMMAND} -DBENCH=$<TARGET_FILE:spacecal-bench> -DWORK=${CMAKE_CURRENT_BINARY_DIR} -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_bench.cmake)
