// STL
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// ROS2 TensorRT
#include "ros2_tensorrt.hpp"

int main(int argc, char **argv) {
  // Initialise ROS2
  rclcpp::init(argc, argv);

  ros2trt_config config;
  config.handle_images = false;
  config.node_name = "test_node";
  config.sub_topic = "array_topic";
  config.pub_topic = "output_topic";
  config.model_path = "/home/aaron/Data/engines/resnet50v1.engine";

  rclcpp::spin(std::make_shared<Ros2TensorrtModule>(config));
  rclcpp::shutdown();
  return 0;
}