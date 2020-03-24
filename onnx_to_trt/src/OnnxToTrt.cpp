#include <iostream>
#include <memory>

#include <NvInfer.h>
#include <NvOnnxParser.h>

// Logger for TensorRT info/warning/errors
class Logger : public nvinfer1::ILogger {
 public:
  Logger(Severity severity = Severity::kWARNING)
      : reportableSeverity(severity) {}

  void log(Severity severity, const char* msg) override {
    // suppress messages with severity enum value greater than the reportable
    if (severity > reportableSeverity) return;

    switch (severity) {
      case Severity::kINTERNAL_ERROR:
        std::cerr << "INTERNAL_ERROR: ";
        break;
      case Severity::kERROR:
        std::cerr << "ERROR: ";
        break;
      case Severity::kWARNING:
        std::cerr << "WARNING: ";
        break;
      case Severity::kINFO:
        std::cerr << "INFO: ";
        break;
      default:
        std::cerr << "UNKNOWN: ";
        break;
    }
    std::cerr << msg << std::endl;
  }

  Severity reportableSeverity;
};

struct InferDeleter {
  template <typename T>
  void operator()(T* obj) const {
    if (obj) {
      obj->destroy();
    }
  }
};

template <typename T>
inline std::shared_ptr<T> infer_object(T* obj) {
  if (!obj) {
    throw std::runtime_error("Failed to create object");
  }
  return std::shared_ptr<T>(obj, InferDeleter());
}

int main(int argc, char** argv)
{
  Logger g_logger_;
  int verbosity_ = (int)nvinfer1::ILogger::Severity::kWARNING;

  std::shared_ptr<nvinfer1::IBuilder> trt_builder_;
  std::shared_ptr<nvinfer1::INetworkDefinition> trt_network_;
  std::shared_ptr<nvonnxparser::IParser> trt_parser_;
  std::shared_ptr<nvinfer1::ICudaEngine> trt_engine_;
  std::shared_ptr<nvinfer1::IRuntime> trt_runtime_;
  std::shared_ptr<nvinfer1::IExecutionContext> trt_context_;

  trt_builder_ = infer_object(nvinfer1::createInferBuilder(g_logger_));

  std::cout << "all done" << std::endl;
  return 0;
}
