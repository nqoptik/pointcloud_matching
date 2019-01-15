cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(pointcloud_matching)

find_package(PCL 1.8.1 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PreProcess
    src/preprocess.cpp
    src/Configurations.cpp
)
target_link_libraries(PreProcess
    ${PCL_LIBRARIES}
    las
)

add_executable(Matching
    src/matching.cpp
    src/Configurations.cpp
    src/CloudProjection.cpp
)
target_link_libraries(Matching
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(ReferPlane
    src/ReferPlane.cpp
    src/CloudDiffChecker.cpp
)
target_link_libraries(ReferPlane
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)
