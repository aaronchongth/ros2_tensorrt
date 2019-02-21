#include "test_subscriber.hpp"

TestSubscriberConfig collect_config(const opt::ParseResult &args) {
  TestSubscriberConfig config;
  config.node_name = std::string(args["node-name"].as<std::string>());
  config.sub_topic = std::string(args["sub-topic"].as<std::string>());
  return config;
}

int main(int argc, char **argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Get the config parameters
  TestSubscriberConfig config;
  opt::Options options("test_subscriber",
                       "Subscribes to the output of the TensorRT inference "
                       "results, and tests and output.");
  options.add_options()(
      "node-name", "Node name",
      opt::value<std::string>()->default_value(config.node_name))(
      "sub-topic", "Subscription topic for array",
      opt::value<std::string>()->default_value(config.sub_topic))(
      "h, help", "Displayes help message and lists arguments",
      opt::value<std::string>()->default_value("false"));

  bool help = false;
  try {
    auto args = options.parse(argc, argv);
    if (args["help"].as<std::string>() == "true")
      help = true;
    else {
      TestSubscriberConfig init_config = collect_config(args);
      rclcpp::spin(std::make_shared<TestSubscriber>(init_config));
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

  std::cout << "<STATUS> Test inference result subscriber done." << std::endl;
  return 0;
}