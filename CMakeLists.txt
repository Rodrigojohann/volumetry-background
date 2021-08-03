
project(VolumeVisionaryT)
cmake_minimum_required(VERSION 2.8)
find_package(PCL 1.2 REQUIRED)

### COMPILER FLAGS ###
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

### BUILD ###

include_directories(VisionaryCommon VisionaryCommon/include)
add_subdirectory(VisionaryCommon)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Visionary-S sample ##

## Visionary-T sample ##
add_executable(write-background write-background.cpp)
target_link_libraries(write-background sick_visionary_common ${PCL_LIBRARIES})

