cmake_minimum_required(VERSION 3.10)

# 设置项目名称和版本
project(optimize VERSION 1.0)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV version ${OpenCV_VERSION}")
else()
    message(FATAL_ERROR "OpenCV not found")
endif()

# 查找Eigen库
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
if (Eigen3_FOUND)
    message(STATUS "Found Eigen3")
else()
    message(FATAL_ERROR "Eigen3 not found")
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Ceres_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
add_subdirectory(manual_optimizer)
add_subdirectory(slam_optimizer_ceres)
add_subdirectory(utils)