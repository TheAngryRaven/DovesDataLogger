cmake_minimum_required(VERSION 3.14)
project(birdseye_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Pure-logic source extracted from the Arduino sketch. These files
# must be free of Arduino headers so they compile on a host toolchain.
set(BIRDSEYE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../BirdsEye)

add_executable(birdseye_tests
  test_main.cpp
  haversine_test.cpp
  gps_time_test.cpp
  ${BIRDSEYE_DIR}/haversine.cpp
  ${BIRDSEYE_DIR}/gps_time.cpp
)

target_include_directories(birdseye_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}  # for doctest.h
  ${BIRDSEYE_DIR}              # for the extracted module headers
)

# doctest.h is third-party; suppress warnings it triggers without
# muting them for our own code.
set_source_files_properties(test_main.cpp PROPERTIES
  COMPILE_FLAGS "-w"
)

target_compile_options(birdseye_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

enable_testing()
add_test(NAME birdseye_tests COMMAND birdseye_tests)
