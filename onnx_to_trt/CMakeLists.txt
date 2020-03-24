cmake_minimum_required(VERSION 3.5.0)
project(onnx_to_trt VERSION 0.0.1 LANGUAGES C CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include(GNUInstallDirs)

# TensorRT
find_path(TensorRT_INCLUDE_DIR NvInfer.h
  HINTS $ENV{TRT_RELEASE}
  PATH_SUFFIXES include)
find_library(TensorRT_LIBRARY_INFER nvinfer
  HINTS $ENV{TRT_RELEASE}
  PATH_SUFFIXES lib lib64 lib/x64)
find_library(TensorRT_LIBRARY_ONNX nvonnxparser
  HINTS $ENV{TRT_RELEASE}
  PATH_SUFFIXES lib lib64 lib/x64)
set (TensorRT_LIBRARIES ${TensorRT_LIBRARY_INFER} ${TensorRT_LIBRARY_ONNX})
MESSAGE(STATUS "Find TensorRT libs at ${TENSORRT_LIBRARY}")
# find_package_handle_standard_args(
#   TensorRT DEFAULT_MSG TensorRT_INCLUDE_DIR TensorRT_LIBRARIES)
# if(NOT TensorRT_FOUND)
#   message(ERROR "Cannot find TensorRT library.")
# endif()

add_executable(${PROJECT_NAME}_utils
  src/OnnxToTrt.cpp
)
target_include_directories(${PROJECT_NAME}_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${TensorRT_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}_utils
  ${TensorRT_LIBRARIES}
)
