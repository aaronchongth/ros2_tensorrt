#pragma once

// STL
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

// TensorRT
#include "tensorrt_base.hpp"

// CXXOPTS
#include "cxxopts/cxxopts.hpp"
namespace opt = cxxopts;

struct TestSubscriberConfig {
  std::string node_name = "test_inference_results_node";
  std::string sub_topic = "output_topic";
  int gt_label = 281;
};

class TestSubscriber : public rclcpp::Node {
 public:
  explicit TestSubscriber(const TestSubscriberConfig &config)
      : Node(config.node_name),
        sub_topic_(config.sub_topic),
        gt_class_(config.gt_label) {
    test_array_subscriber_ =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            sub_topic_,
            std::bind(&TestSubscriber::test_array_callback, this, _1));

    info_timer_ = this->create_wall_timer(
        1000ms, std::bind(&TestSubscriber::info_timer_callback, this));

    std::cout << "<STATUS> Test inference result subscriber initialized, ready "
                 "for callback inference."
              << std::endl;
  }

 private:
  int gt_class_, pred_class_;
  int n_messages_ = 0;

  rclcpp::TimerBase::SharedPtr info_timer_;

  std::string sub_topic_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      test_array_subscriber_;

  void test_array_callback(
      const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // try to get topK results
    int n_elems = msg->layout.dim[0].size;
    std::vector<float> output(&(msg->data[0]), &(msg->data[0]) + 1000);
    auto top = tensorrt_common::topK<float>(output, 1);
    pred_class_ = top[0];
    n_messages_++;
  }

  void info_timer_callback() {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "<TEST_RESULTS> Rate: "
              << static_cast<float>(1000.0 / n_messages_)
              << "Hz;   Inference Classification: ";
    n_messages_ = 0;

    if (gt_class_ == pred_class_)
      std::cout << "PASS" << std::endl;
    else
      std::cout << "FAILED, pred: " << pred_class_ << "    gt: " << gt_class_
                << std::endl;
  }
};