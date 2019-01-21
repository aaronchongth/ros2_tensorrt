#pragma once

// STL
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// OpenCV
#include "opencv2/opencv.hpp"

// TensorRT
#include "tensorrt_base.hpp"

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

/*
ROS2 TensorRT Module
- initializes and builds a TensorRT engine for inference
- subscribes to a sensor_msg compressed image topic, preprocesses it
- automatically resizes image to suit inference size, simplifies things
- uses OpenCV for encoding and decoding of JPEGs, might swtich to LibJPEGturbo
- allows users to use their own preprocessing standard functions using CV::Mat,
  will write an example function, normalize to Imagenet colours
- ideally runs everything using just a config that can be filled up

Notes:
- sticks to fp32 (experiments show fp16 doesnt speed things up)
- sticks to batch size of 1, and optimization workspace of 1 << 21, unless
  stated
- due to OpenCV, remember it is BGR

TODO:
- think of how the interface should allow users to handle the inference results
- finish implementation until the above points
- check out the quality of service aspect too
- in tensorrt module, check if ext is onnx, batchsize and workspace must be
  provided
*/

struct ros2trt_config {
  // ros2 items
  std::string node_name;
  std::string sub_topic;

  // tensorrt items
  std::string model_path;
  int batch_size = 1;
  int max_workspace = 1 << 21;
};

class Ros2TensorrtModule : public rclcpp::Node {
 public:
  explicit Ros2TensorrtModule(const ros2trt_config &config);
  ~Ros2TensorrtModule();

 private:
  // ROS2 items
  std::string node_name_, sub_topic_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      subscriber_;
  // callback function that handles the inference
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

  // TensorRT Items
  std::string model_path_;
  int batch_size_, max_workspace_;
  std::unique_ptr<TensorRTModule> tensorrt_module_;
};
