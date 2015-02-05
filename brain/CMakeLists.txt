cmake_minimum_required (VERSION 2.6)
project (Brain)

# Define our version number
set (CREATE_VERSION_MAJOR 1)
set (CREATE_VERSION_MINOR 0)
configure_file (
  "${Brain_SOURCE_DIR}/src/create_config.h.in"
  "${Brain_BINARY_DIR}/create_config.h")

# ${skeleton_SOURCE_DIR}
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/src")

include_directories(${Create_SOURCE_DIR}/gtest-1.7.0/include)
link_directories(${Create_SOURCE_DIR}/gtest-1.7.0/src)

set(CREATE_SRCS src/irobot_create src/irobot_create_open_interface src/tools/status)
add_executable(Create src/controller_main ${CREATE_SRCS})
target_link_libraries(Create ${catkin_LIBRARIES} serial)

# Build tests
set(TEST_SRCS src/irobot_create_test)
add_executable(runUnitTests ${TEST_SRCS} ${CREATE_SRCS})
target_link_libraries(runUnitTests gtest gtest_main)
target_link_libraries(runUnitTests ${catkin_LIBRARIES} serial)
add_test(runUnitTests runUnitTests)
