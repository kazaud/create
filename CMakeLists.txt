cmake_minimum_required (VERSION 2.6)
project (Create)

# GTest configuration
add_subdirectory (gtest-1.7.0)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

message(STATUS "Compiler is")
if(CMAKE_COMPILER_IS_GNUCXX)
	message(STATUS "GNUCXX")
	list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -Wall -Wextra -Winit-self -Wno-long-long -Wold-style-cast -Woverloaded-virtual -Wuninitialized -Wmissing-declarations -Winit-self -pedantic -Werror -g -ftest-coverage -fprofile-arcs")
endif()
add_subdirectory (brain brain/build)


# Add a test
#add_executable(runUnitTests skeleton/src/create_test.cc)
#target_link_libraries(runUnitTests gtest gtest_main)
#add_test(runUnitTests runUnitTests)
