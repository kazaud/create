cmake_minimum_required (VERSION 2.6)

# Define our version number
set (CREATE_VERSION_MAJOR 1)
set (CREATE_VERSION_MINOR 0)
configure_file (
  "${CMAKE_CURRENT_LIST_DIR}/src/create_config.h.in"
  "${PROJECT_BINARY_DIR}/create_config.h")

include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/src")

add_executable(Create src/create.cxx)
