cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(PointcloudMatching)

find_package(PCL 1.4 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (PreProcess preprocess.cpp preprocess.h Configurations.cpp Configurations.h)
add_executable (Matching matching.cpp matching.h Configurations.cpp Configurations.h CloudProjection.h CloudProjection.cpp)
add_executable (ReferPlane ReferPlane.cpp CloudDiffChecker.h CloudDiffChecker.cpp)

target_link_libraries (PreProcess ${PCL_LIBRARIES} las)
target_link_libraries (Matching ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries (ReferPlane ${PCL_LIBRARIES} ${OpenCV_LIBS})

