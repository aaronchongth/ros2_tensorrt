#pragma once

// STL
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// OpenCV
#include "opencv2/opencv.hpp"

// TensorRT
#include "tensorrt_base.hpp"

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

/*
ROS2 TensorRT Module
- initializes and builds a TensorRT engine for inference
- subscribes to a Float32MultiArray topic, inferences directly, and publishes
inference results
- Runs everything using just a config that can be filled up

Notes:
- sticks to fp32 (experiments show fp16 doesnt speed things up)
- sticks to batch size of 1, and optimization workspace of 1 << 21, unless
  stated

TODO:
- due to OpenCV, remember it is BGR
- uses OpenCV for encoding and decoding of JPEGs, might swtich to LibJPEGturbo
- allows users to use their own preprocessing standard functions using CV::Mat,
  will write an example function, normalize to Imagenet colours
- automatically resizes image to suit inference size, simplifies things
- subscribes to a sensor_msg compressed image topic, preprocesses it
- check out the quality of service aspect too
*/

struct Ros2TensorrtModuleConfig {
  // ros2 items
  std::string node_name = "ros2inference";
  std::string sub_topic = "sub_topic";
  std::string pub_topic = "pub_topic";

  // tensorrt items
  std::string model_path = "model_path";
  int batch_size = 1;
  int max_workspace = 1 << 21;
};

class Ros2TensorrtModule : public rclcpp::Node {
 public:
  explicit Ros2TensorrtModule(const Ros2TensorrtModuleConfig &config);
  ~Ros2TensorrtModule();

 private:
  // ROS2 items
  std::string node_name_, sub_topic_, pub_topic_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      array_subscriber_;
  void array_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      array_publisher_;

  rclcpp::TimerBase::SharedPtr info_timer_;
  int n_messages_ = 0;
  void info_timer_callback();

  // TensorRT Items
  std::string model_path_;
  int batch_size_, max_workspace_;
  std::unique_ptr<TensorRTModule> tensorrt_module_;
  uint32_t input_channels_, input_rows_, input_cols_;
};
