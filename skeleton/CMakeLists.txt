cmake_minimum_required (VERSION 2.6)

# Define our version number
set (CREATE_VERSION_MAJOR 1)
set (CREATE_VERSION_MINOR 0)
configure_file (
  "${CMAKE_CURRENT_LIST_DIR}/src/create_config.h.in"
  "${PROJECT_BINARY_DIR}/create_config.h")

# ${skeleton_SOURCE_DIR}
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/src")

include_directories(${Create_SOURCE_DIR}/gtest-1.7.0/include)
link_directories(${Create_SOURCE_DIR}/gtest-1.7.0/src)


add_executable(Create src/create_main.cc src/create.cpp src/create.h)

# Add a test
add_executable(runUnitTests src/create_test.cc src/create.cpp src/create.h)
target_link_libraries(runUnitTests gtest gtest_main)
add_test(runUnitTests runUnitTests)
