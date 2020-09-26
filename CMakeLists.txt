cmake_minimum_required(VERSION 3.0.0)
project(lidar_hist VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(lidar_hist src/main.cpp src/projectPointCloud.cpp src/histogram.cpp src/hist_filter.cpp src/ransac.cpp src/split3.cpp)

target_link_libraries(lidar_hist ${PCL_LIBRARIES})
target_link_libraries(lidar_hist ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
