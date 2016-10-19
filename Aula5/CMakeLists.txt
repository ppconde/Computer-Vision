CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( calib )
MESSAGE ("\n==> Finding 3rd Party Packages")
find_package( OpenCV REQUIRED )
#find_package(PCL 1.2 REQUIRED)

SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -W" )
SET( CMAKE_C_FLAGS_RELEASE "-O3" )
SET( CMAKE_C_FLAGS_DEBUG "-g3" )

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall" )
SET( CMAKE_CXX_FLAGS_RELEASE "-O3" )
SET( CMAKE_CXX_FLAGS_DEBUG "-g3" )

SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )

# Make include directories available for all..
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIR} )

# Add sources directory
ADD_SUBDIRECTORY( src )
