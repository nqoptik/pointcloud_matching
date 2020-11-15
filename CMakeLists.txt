cmake_minimum_required(VERSION 3.5)

project(pointcloud_matching)

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Compile with the highest warning level
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

## System dependencies
find_package(PCL 1.7 REQUIRED)

find_package(OpenCV REQUIRED)
if(NOT ${OpenCV_VERSION} STRGREATER "2.4")
    message(FATAL_ERROR "OpenCV_VERSION has to > 2.4")
endif()

## Specify additional locations of header files
include_directories(${PCL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)

## Declare a C++ executable
add_executable(matching src/cloud_projection.cpp src/configurations.cpp
                        src/matching.cpp)

add_executable(pre_process src/configurations.cpp src/preprocess.cpp)

add_executable(refer_plane src/cloud_diff_checker.cpp src/refer_plane.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(matching ${OpenCV_LIBS} ${PCL_LIBRARIES})

target_link_libraries(pre_process ${PCL_LIBRARIES} las)

target_link_libraries(refer_plane ${OpenCV_LIBS} ${PCL_LIBRARIES})
