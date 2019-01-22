#include "ros2_tensorrt.hpp"

Ros2TensorrtModule::Ros2TensorrtModule(const ros2trt_config &config)
    : Node(config.node_name),
      sub_topic_(config.sub_topic),
      model_path_(config.model_path),
      batch_size_(config.batch_size),
      max_workspace_(config.max_workspace) {
  if (strcmp(tensorrt_common::getFileType(model_path_).c_str(), "onnx") == 0)
    tensorrt_module_ = std::make_unique<TensorRTModule>(
        model_path_, batch_size_, max_workspace_);
  else
    tensorrt_module_ = std::make_unique<TensorRTModule>(model_path_);

  std::cout << "<STATUS> ROS2 TensorRT Module initialized, ready for callback "
               "inference."
            << std::endl;
}

Ros2TensorrtModule::~Ros2TensorrtModule() {
  std::cout << "<STATUS> ROS2 TensorRT Module killed." << std::endl;
}

void Ros2TensorrtModule::image_callback(
    const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  std::cout << "<STATUS> Callback inference called." << std::endl;
}