// STL
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// TensorRT
#include "tensorrt_base.hpp"

int main(int argc, char **argv) {
  // preparing the module on device memory
  std::string engine_path = "data/resnet50v1.engine";
  int batch_size = 1;
  std::unique_ptr<TensorRTModule> tensorrt_module =
      std::make_unique<TensorRTModule>(engine_path, batch_size);

  // preparing the dummy input
  std::vector<std::vector<float>> dummy_input_buffer;
  std::vector<float> dummy_input(3 * 224 * 224, 0.5);
  dummy_input_buffer.push_back(dummy_input);

  // Inference benchmarking
  int n_iters = 3000;
  int warmup_iters = 2000;
  long accum_ms = 0;
  int num = 0;
  std::cout << "<STATUS> Inference benchmarking started." << std::endl;
  for (int i = 0; i < n_iters; i++) {
    auto t0 = std::chrono::system_clock::now();
    tensorrt_module->inference(dummy_input_buffer);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now() - t0);
    if (i + 1 > warmup_iters) accum_ms += static_cast<long>(duration.count());
  }

  float avg_ms = static_cast<float>(accum_ms) / 1000.0;
  std::cout << "<STATUS> Benchmarking 1000 iterations, Average: " << avg_ms
            << " ms, " << 1000.0 / avg_ms << " Hz" << std::endl;
  std::cout << "<STATUS> Inference benchmarking done." << std::endl;
  return 0;
}