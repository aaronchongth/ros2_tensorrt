#pragma once

// STL
#include <functional>
#include <iostream>
#include <memory>

// CUDA
#include "cuda_runtime_api.h"

// TensorRT
#include "NvInfer.h"

#define CHECK(status)                       \
  do {                                      \
    auto ret = (status);                    \
    if (ret != 0) {                         \
      std::cout << "Cuda failure: " << ret; \
      abort();                              \
    }                                       \
  } while (0)

// Custom CUDA mallocs and deleters
inline void* host_pinned_malloc(size_t size) {
  void* ptr;
  CHECK(cudaMallocHost((void**)&ptr, size));
  return ptr;
}

inline void* device_malloc(size_t size) {
  void* ptr;
  CHECK(cudaMalloc((void**)&ptr, size));
  return ptr;
}

inline void host_pinned_deleter(void* ptr) {
  CHECK(cudaFreeHost(ptr));
  std::cout << "Host pinned memory freed." << std::endl;
}

inline void device_deleter(void* ptr) {
  CHECK(cudaFree(ptr));
  std::cout << "Device memory freed." << std::endl;
};

std::unique_ptr<void, std::function<void(void*)>> cuda_malloc_from_dims(
    nvinfer1::Dims dims);

std::unique_ptr<void, std::function<void(void*)>> cuda_host_malloc_from_dims(
    nvinfer1::Dims dims);