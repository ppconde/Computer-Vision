cmake_minimum_required(VERSION 2.8)
project(ProjectName)
message("\n==> Finding 3rd Party Packages")
find_package(OpenCV REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)	
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -W")
set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_C_FLAGS_DEBUG "-g3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-g3")
set(BASE_DIR ${CMAKE_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin)
find_package(Boost 1.64.0 COMPONENTS system filesystem REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR} ${PYTHON_INCLUDE_DIR} ${Boost_SYSTEM_LIBRARIES} ${Boost_FILESYSTEM_LIBRARIES})
add_subdirectory( src )
file(GLOB MY_SOURCE_FILES
	"*.h"
	"*.hpp"
	"*.cpp"
)
add_executable( ProjectName ${MY_SOURCE_FILES} )
