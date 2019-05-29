cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(pointcloud_matching)

find_package(PCL 1.8.1 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(Matching src/CloudProjection.cpp src/Configurations.cpp
                        src/matching.cpp)
target_link_libraries(Matching ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(PreProcess src/Configurations.cpp src/preprocess.cpp)
target_link_libraries(PreProcess ${PCL_LIBRARIES} las)

add_executable(ReferPlane src/CloudDiffChecker.cpp src/ReferPlane.cpp)
target_link_libraries(ReferPlane ${OpenCV_LIBS} ${PCL_LIBRARIES})
