#pragma once

// STL
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// CUDA
#include "cuda_runtime_api.h"

// TensorRT
#include "NvInfer.h"
#include "NvInferPlugin.h"
#include "NvOnnxConfig.h"
#include "NvOnnxParser.h"

// custom
#include "common.h"

// make a module that can handle both engine files and onnx model files
// first, handle onnx models
// user will point directly to the path of the model
// currently handle batch size 1
class TensorRTModule {
 public:
  explicit TensorRTModule(std::string model_path, int batch_size,
                          int max_workspace_size);
  explicit TensorRTModule(std::string model_path);
  ~TensorRTModule();
  bool inference(const std::vector<std::vector<float>> &input);
  const std::vector<float> &get_output(int output_index);
  void save_engine(std::string engine_path);
  void load_engine();

  // get functions
  std::string get_model_path() { return model_path_; }
  std::string get_model_type() { return model_type_; }
  int get_n_inputs() { return n_inputs_; }
  int get_n_outputs() { return n_outputs_; }
  nvinfer1::Dims get_input_dimensions(int input_index) {
    if (input_index >= n_inputs_) {
      std::string msg(
          "Requested input index larger than actual number of model inputs.");
      g_logger_.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
      exit(EXIT_FAILURE);
    }
    return input_dims_[input_index];
  }
  nvinfer1::Dims get_output_dimensions(int output_index) {
    if (output_index >= n_outputs_) {
      std::string msg(
          "Requested output index larger than actual number of model outputs.");
      g_logger_.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
      exit(EXIT_FAILURE);
    }
    return output_dims_[output_index];
  }

 private:
  std::string model_path_;
  std::string model_type_;
  int batch_size_, max_workspace_size_;

  // TensorRT loggers and default parameters
  Logger g_logger_;
  int verbosity_ = (int)nvinfer1::ILogger::Severity::kWARNING;

  // TensorRT objects
  std::shared_ptr<nvinfer1::IBuilder> trt_builder_;
  std::shared_ptr<nvinfer1::INetworkDefinition> trt_network_;
  std::shared_ptr<nvonnxparser::IParser> trt_parser_;
  std::shared_ptr<nvinfer1::ICudaEngine> trt_engine_;
  std::shared_ptr<nvinfer1::IRuntime> trt_runtime_;
  std::shared_ptr<nvinfer1::IExecutionContext> trt_context_;
  cudaStream_t stream_;

  // TensorRT network details
  int n_inputs_, n_outputs_, n_layers_;
  std::vector<nvinfer1::Dims> input_dims_, output_dims_;
  std::vector<int64_t> input_vols_, output_vols_;

  // TensorRT engine details
  int n_bindings_;
  std::vector<std::unique_ptr<void, std::function<void(void *)>>>
      engine_smart_mem_;
  std::vector<void *> engine_mem_;
  std::vector<std::vector<float>> output_mem_;

  // helper functions
  void runtime_error_(std::string error_msg) {
    g_logger_.log(nvinfer1::ILogger::Severity::kERROR, error_msg.c_str());
    exit(EXIT_FAILURE);
  }
};

void print_dims(nvinfer1::Dims dimensions);