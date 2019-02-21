// STL
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// CXXOPTS
#include "cxxopts/cxxopts.hpp"
namespace opt = cxxopts;

// ROS2 TensorRT
#include "ros2_tensorrt.hpp"

// collect config function
Ros2TensorrtModuleConfig collect_config(const opt::ParseResult &args) {
  Ros2TensorrtModuleConfig config;
  config.node_name = std::string(args["node-name"].as<std::string>());
  config.sub_topic = std::string(args["sub-topic"].as<std::string>());
  config.pub_topic = std::string(args["pub-topic"].as<std::string>());

  config.model_path = std::string(args["model-path"].as<std::string>());
  config.batch_size = static_cast<int>(args["batch-size"].as<int>());
  config.max_workspace = static_cast<int>(args["max-workspace"].as<int>());
  return config;
}

int main(int argc, char **argv) {
  // Initialise ROS2
  rclcpp::init(argc, argv);

  // Get the config parameters
  Ros2TensorrtModuleConfig config;
  opt::Options options(
      "ros2inference",
      "Subscribes to a Float32MultiArray type message, inferences it in a "
      "TensorRT engine, publishes the output as another message.");
  options.add_options()(
      "node-name", "Node name",
      opt::value<std::string>()->default_value(config.node_name))(
      "sub-topic", "Subscription topic name",
      opt::value<std::string>()->default_value(config.sub_topic))(
      "pub-topic", "Publishing topic name",
      opt::value<std::string>()->default_value(config.pub_topic))(
      "model-path", "Model path, .engine or .onnx",
      opt::value<std::string>()->default_value(config.model_path))(
      "batch-size", "Inference batch size, normally 1",
      opt::value<int>()->default_value(std::to_string(config.batch_size)))(
      "max-workspace", "GPU workspace allowed for use in optimization",
      opt::value<int>()->default_value(std::to_string(config.max_workspace)))(
      "h, help", "Displays help message and lists arguments",
      opt::value<std::string>()->default_value("false"));

  bool help = false;
  try {
    auto args = options.parse(argc, argv);
    if (args["help"].as<std::string>() == "true")
      help = true;
    else {
      Ros2TensorrtModuleConfig init_config = collect_config(args);
      rclcpp::spin(std::make_shared<Ros2TensorrtModule>(init_config));
      rclcpp::shutdown();
    }
  } catch (const opt::OptionException &e) {
    std::cout << "<ERROR> Option exception." << std::endl;
    help = true;
  }

  if (help) {
    std::cout << options.help() << std::endl;
    return 0;
  }
}