cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(pointcloud_matching)

find_package(PCL 1.8.1 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(matching src/cloud_projection.cpp src/configurations.cpp
                        src/matching.cpp)
target_link_libraries(matching ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(pre_process src/configurations.cpp src/preprocess.cpp)
target_link_libraries(pre_process ${PCL_LIBRARIES} las)

add_executable(refer_plane src/cloud_diff_checker.cpp src/refer_plane.cpp)
target_link_libraries(refer_plane ${OpenCV_LIBS} ${PCL_LIBRARIES})
