cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(PointcloudMatching)

find_package(PCL 1.4 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (main main.cpp plyio.h plyio.cpp)

target_link_libraries (main ${PCL_LIBRARIES} ${OpenCV_LIBS})

