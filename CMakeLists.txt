cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(pointcloud_matching)

find_package(PCL 1.4 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PreProcess preprocess.cpp Configurations.cpp)
add_executable(Matching matching.cpp Configurations.cpp CloudProjection.cpp)
add_executable(ReferPlane ReferPlane.cpp CloudDiffChecker.cpp)

target_link_libraries(PreProcess ${PCL_LIBRARIES} las)
target_link_libraries(Matching ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(ReferPlane ${PCL_LIBRARIES} ${OpenCV_LIBS})
