cmake_minimum_required(VERSION 3.3 FATAL_ERROR)

# Must be C++14
set (CMAKE_CXX_STANDARD 14)

#-------------------------------------------------------------------------------
# Project setup
#-------------------------------------------------------------------------------
project(tensorrt_pipeline C CXX)
set(CMAKE_BINARY_DIR "${CMAKE_SOURCE_DIR}/bin")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

#-------------------------------------------------------------------------------
# CUDA
#-------------------------------------------------------------------------------
find_package(CUDA REQUIRED)

#-------------------------------------------------------------------------------
# TensorRT
#-------------------------------------------------------------------------------
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS $ENV{TENSORRT_DIR}
  PATH_SUFFIXES include)
find_library(TENSORRT_LIBRARY_INFER nvinfer
  HINTS $ENV{TENSORRT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64)
find_library(TENSORRT_LIBRARY_ONNX nvonnxparser
  HINTS $ENV{TENSORRT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64)
set (TENSORRT_LIBRARY ${TENSORRT_LIBRARY_INFER} ${TENSORRT_LIBRARY_ONNX})
MESSAGE(STATUS "Find TensorRT libs at ${TENSORRT_LIBRARY}")
find_package_handle_standard_args(
  TENSORRT DEFAULT_MSG TENSORRT_INCLUDE_DIR TENSORRT_LIBRARY)
if(NOT TENSORRT_FOUND)
  message(ERROR
    "Cannot find TensorRT library.")
endif()

#-------------------------------------------------------------------------------
# ROS 2
#-------------------------------------------------------------------------------
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

#-------------------------------------------------------------------------------
# OpenCV
#-------------------------------------------------------------------------------
set(OpenCV_BUILD_DIR "${CMAKE_SOURCE_DIR}/deps/opencv/build")
find_package(OpenCV 3.4.0 REQUIRED HINTS ${OpenCV_BUILD_DIR})

#-------------------------------------------------------------------------------
# Eigen
#-------------------------------------------------------------------------------
set(eigen_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rd_party/eigen")

#-------------------------------------------------------------------------------
# cxxopts
#-------------------------------------------------------------------------------
set(cxxopts_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/cxxopts/include")

#-------------------------------------------------------------------------------
# Library targets
#-------------------------------------------------------------------------------
set(TENSORRT_BASE_DIR "${CMAKE_SOURCE_DIR}/src/tensorrt_base")
add_subdirectory(${TENSORRT_BASE_DIR})
set(ROS2_BASE_DIR "${CMAKE_SOURCE_DIR}/src/ros2_base")
add_subdirectory(${ROS2_BASE_DIR})

#-------------------------------------------------------------------------------
# Individual driver targets
#-------------------------------------------------------------------------------
add_subdirectory(src/test_libs)
