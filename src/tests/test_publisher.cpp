#include "test_publisher.hpp"

TestPublisherConfig collect_config(const opt::ParseResult &args) {
  TestPublisherConfig config;
  config.image_path = std::string(args["image-path"].as<std::string>());
  config.node_name = std::string(args["node-name"].as<std::string>());
  config.jpg_topic = std::string(args["jpg-topic"].as<std::string>());
  config.array_topic = std::string(args["array-topic"].as<std::string>());
  return config;
}

int main(int argc, char **argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Get the config parameters
  TestPublisherConfig config;
  opt::Options options("test_publisher",
                       "Publishes a test image in CompressedImage and "
                       "Float32MultiArray formats.");
  options.add_options()(
      "image-path", "Path to image",
      opt::value<std::string>()->default_value(config.image_path))(
      "node-name", "Publisher node name",
      opt::value<std::string>()->default_value(config.node_name))(
      "jpg-topic", "JPG publisher topic",
      opt::value<std::string>()->default_value(config.jpg_topic))(
      "array-topic", "Float32MultiArray publisher topic",
      opt::value<std::string>()->default_value(config.array_topic))(
      "h, help", "Displays help message and lists arguments",
      opt::value<std::string>()->default_value("false"));

  bool help = false;
  try {
    auto args = options.parse(argc, argv);
    if (args["help"].as<std::string>() == "true")
      help = true;
    else {
      TestPublisherConfig init_config = collect_config(args);
      rclcpp::spin(std::make_shared<TestPublisher>(init_config));
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

  std::cout << "<STATUS> Test publisher done." << std::endl;
  return 0;
}