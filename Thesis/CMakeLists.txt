cmake_minimum_required(VERSION 2.8)
project(ProjectName)
message("\n==> Finding 3rd Party Packages")
find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -W")
set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_C_FLAGS_DEBUG "-g3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-g3")
set(BASE_DIR ${CMAKE_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin)

#include_directories(${OpenCV_INCLUDE_DIR})
add_subdirectory( src )

