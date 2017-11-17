cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(PointcloudMatching)

find_package(PCL 1.4 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (main main.cpp plyio.h plyio.cpp Configurations.cpp Configurations.h)
add_executable (PreProcessing PreProcessing.cpp Configurations.cpp Configurations.h)
add_executable (Projection2D Projection2D.cpp plyio.h plyio.cpp Configurations.cpp Configurations.h)

target_link_libraries (main ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries (PreProcessing ${PCL_LIBRARIES} las)
target_link_libraries (Projection2D ${PCL_LIBRARIES} ${OpenCV_LIBS})
