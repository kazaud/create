cmake_minimum_required (VERSION 2.6)
SET(CMAKE_CXX_FLAGS "-std=c++0x")
project (Create)

# GTest configuration
add_subdirectory (gtest-1.7.0)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_subdirectory (skeleton skeleton/build)


# Add a test
#add_executable(runUnitTests skeleton/src/create_test.cc)
#target_link_libraries(runUnitTests gtest gtest_main)
#add_test(runUnitTests runUnitTests)
