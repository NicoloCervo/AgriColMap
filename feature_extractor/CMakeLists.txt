cmake_minimum_required( VERSION 2.8 )

project( feature_extractor )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

add_executable( feature_extractor main.cpp )

# Find Packages
find_package( PCL 1.8 REQUIRED )

find_package( OpenCV 4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( feature_extractor ${OpenCV_LIBS} )

if( PCL_FOUND )

    include_directories( ${PCL_INCLUDE_DIRS} )
    add_definitions( ${PCL_DEFINITIONS} )
    link_directories( ${PCL_LIBRARY_DIRS} )
    target_link_libraries( feature_extractor ${PCL_LIBRARIES} )

endif()